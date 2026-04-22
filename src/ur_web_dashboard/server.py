#!/usr/bin/env python3
"""Simple HTTP server that serves the web dashboard and UR description meshes."""
import http.server
import os
import sys

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
ROSBRIDGE_PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 9090
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Resolve the active control_mode so the dashboard can gate controller
# switch buttons. Position mode exposes only position actuators in the
# MuJoCo MJCF, so velocity / effort controllers either silently no-op
# or refuse to activate; the dashboard should reflect that.
def _detect_control_mode():
    env = os.environ.get('UR_SIM_CONTROL_MODE')
    if env:
        return env
    # Fall back to reading config/config.yaml.
    ws = os.path.dirname(os.path.dirname(SCRIPT_DIR))
    cfg = os.path.join(ws, 'config', 'config.yaml')
    try:
        with open(cfg) as f:
            for line in f:
                s = line.strip()
                if s.startswith('control_mode:'):
                    return s.split(':', 1)[1].strip().strip('"\'')
    except Exception:
        pass
    return 'position'
CONTROL_MODE = _detect_control_mode()

# Resolve ur_description mesh directory: prefer system ROS package, fall back to submodule
def _find_ur_description_dir():
    """Find ur_description share directory from system package or submodule."""
    try:
        import subprocess
        result = subprocess.run(
            ['ros2', 'pkg', 'prefix', 'ur_description'],
            capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            d = os.path.join(result.stdout.strip(), 'share', 'ur_description')
            if os.path.isdir(d):
                return d
    except Exception:
        pass
    workspace = os.path.dirname(os.path.dirname(SCRIPT_DIR))
    return os.path.join(workspace, 'src', 'ur_description')

UR_DESCRIPTION_DIR = _find_ur_description_dir()


class URDashboardHandler(http.server.SimpleHTTPRequestHandler):
    def translate_path(self, path):
        # Serve /config.json with runtime configuration
        if path == '/config.json':
            return None  # handled in do_GET
        # Serve /ur_description/* from system package or submodule
        if path.startswith('/ur_description/'):
            rel = path[len('/ur_description/'):]
            return os.path.join(UR_DESCRIPTION_DIR, rel)
        # Everything else from the dashboard directory
        return os.path.join(SCRIPT_DIR, path.lstrip('/'))

    def do_GET(self):
        if self.path == '/config.json':
            import json
            config = json.dumps({
                'rosbridge_port': ROSBRIDGE_PORT,
                'rosbridge_url': f'ws://{self.headers.get("Host", "localhost").split(":")[0]}:{ROSBRIDGE_PORT}',
                'control_mode': CONTROL_MODE,
            })
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', len(config))
            self.end_headers()
            self.wfile.write(config.encode())
            return
        super().do_GET()

    def end_headers(self):
        # CORS headers for local dev
        self.send_header('Access-Control-Allow-Origin', '*')
        super().end_headers()

    def log_message(self, format, *args):
        # Quieter logging - only errors
        if args and isinstance(args[0], str) and args[0].startswith('GET'):
            return
        super().log_message(format, *args)


if __name__ == '__main__':
    os.chdir(SCRIPT_DIR)
    with http.server.HTTPServer(('0.0.0.0', PORT), URDashboardHandler) as httpd:
        print(f'UR Dashboard server on http://0.0.0.0:{PORT}')
        print(f'  Dashboard: {SCRIPT_DIR}')
        print(f'  Meshes:    {UR_DESCRIPTION_DIR}')
        httpd.serve_forever()
