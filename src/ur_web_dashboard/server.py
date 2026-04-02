#!/usr/bin/env python3
"""Simple HTTP server that serves the web dashboard and UR description meshes."""
import http.server
import os
import sys

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WORKSPACE = os.path.dirname(os.path.dirname(SCRIPT_DIR))  # ur_sim root


class URDashboardHandler(http.server.SimpleHTTPRequestHandler):
    def translate_path(self, path):
        # Serve /ur_description/* from the ur_description submodule
        if path.startswith('/ur_description/'):
            rel = path[len('/ur_description/'):]
            return os.path.join(WORKSPACE, 'src', 'ur_description', rel)
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
        print(f'  Meshes:    {os.path.join(WORKSPACE, "src", "ur_description")}')
        httpd.serve_forever()
