from http.server import HTTPServer, BaseHTTPRequestHandler
import json
from drone_manager import drone_manager
import os

class DroneHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            with open('index.html', 'rb') as file:
                self.wfile.write(file.read())
        elif self.path == '/drone_data':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(drone_manager.get_data().encode())
        else:
            self.send_error(404)

    def do_POST(self):
        content_length = int(self.headers['Content-Length'])
        post_data = self.rfile.read(content_length)
        data = json.loads(post_data.decode('utf-8'))

        response = {}
        if self.path == '/arm':
            response = drone_manager.send_command('command_long_send', drone_manager.vehicle.target_system, drone_manager.vehicle.target_component, 400, 0, 1, 0, 0, 0, 0, 0, 0)
        elif self.path == '/takeoff':
            altitude = data.get('altitude', 10)
            response = drone_manager.send_command('command_long_send', drone_manager.vehicle.target_system, drone_manager.vehicle.target_component, 22, 0, 0, 0, 0, 0, 0, 0, altitude)
        # Add more command handlers here

        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps(response).encode())

def run_server(port=8000):
    server_address = ('', port)
    httpd = HTTPServer(server_address, DroneHandler)
    print(f'Server running on port {port}')
    httpd.serve_forever()

if __name__ == '__main__':
    drone_manager.start()
    run_server()