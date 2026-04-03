#!/usr/bin/env python3
"""Simple HTTP server that serves the web dashboard and UR description meshes."""
import http.server
import os
import sys

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

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
        # Serve /ur_description/* from system package or submodule
        if path.startswith('/ur_description/'):
            rel = path[len('/ur_description/'):]
            return os.path.join(UR_DESCRIPTION_DIR, rel)
        # Everything else from the dashboard directory
        return os.path.join(SCRIPT_DIR, path.lstrip('/'))

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
