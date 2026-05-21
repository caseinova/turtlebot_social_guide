import os
import webbrowser
import http.server
import socketserver
import threading
import time
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from social_robot_interfaces.msg import TspCommand
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

PACKAGE_NAME = 'pepper_hri'


def get_tablet_assets_dir():
    try:
        return os.path.join(get_package_share_directory(PACKAGE_NAME), 'tablet_assets')
    except PackageNotFoundError:
        return os.path.dirname(os.path.abspath(__file__))


# Global safety anchor for the installed package share, with a source-tree fallback.
SCRIPT_DIR = get_tablet_assets_dir()


class TabletBuilderNode(Node):  

    def __init__(self):
        super().__init__('tablet_builder_node')
        
        # Subscribe to the gesture/tour command topic
        self.subscription = self.create_subscription(
            String,
            '/pepper/explain_exhibit',
            self.listener_callback,
            10)
        
        # Publisher for selected tablet tour waypoints.
        self.tsp_command_publisher = self.create_publisher(
            TspCommand,
            '/tsp_command',
            10)

        self.get_logger().info("Tablet Builder Node is ready.")

    def listener_callback(self, msg):
        command = msg.data.strip()
        self.get_logger().info(f"Rebuilding page for command: {command}")
        
        print(command)
        self.build_page(command)

    def build_page(self, command_name):
        # Load the manifest using absolute positioning
        manifest_path = os.path.join(SCRIPT_DIR, 'exhibition_commands.json')
        with open(manifest_path, 'r') as f:
            manifest = json.load(f)

        if command_name not in manifest:
            print(f"Error: Command '{command_name}' not found in exhibition_commands.json")
            return

        config = manifest[command_name]

        # Convert raw config paths to absolute paths for Python filesystem checks
        img_folder_abs = os.path.join(SCRIPT_DIR, config['image_folder'])
        text_file_abs = os.path.join(SCRIPT_DIR, config['text_file'])

        # Name of header derived safely from path base
        folder_name = os.path.basename(img_folder_abs)
        clean_title = folder_name.replace('_', ' ').title()

        # Count how many images are in the target folder safely
        if os.path.exists(img_folder_abs):
            images_list = [f for f in os.listdir(img_folder_abs) if f.lower().endswith(('.png', '.jpg', '.jpeg', '.webp'))]
            num_images = len(images_list)
        else:
            num_images = 0

        print(f"Detected image count: {num_images}")

        # If 3 or more images, turn on the scroll animation
        scroll_class = "animate-scroll" if num_images >= 2 else "static-gallery"
        
        # Read the artifact text file safely
        with open(text_file_abs, 'r') as f:
            description = f.read()

        # Gather all images from the specified folder
        valid_extensions = ('.webp', '.jpeg', '.jpg', '.gif', '.png')
        
        gallery_html = ""
        if os.path.exists(img_folder_abs):
            for filename in sorted(os.listdir(img_folder_abs)):
                if filename.lower().endswith(valid_extensions):
                    # For the browser source tag, keep the path relative to the server root
                    rel_path = os.path.join(config['image_folder'], filename)
                    gallery_html += f'<img src="{rel_path}" style="width:100%; margin-bottom:20px;">\n'
        else:
            print(f"Warning: Folder {img_folder_abs} not found.")

        # Assemble the final HTML using absolute template paths
        layout_path = os.path.join(SCRIPT_DIR, 'layout.html')
        with open(layout_path, 'r') as f:
            template = f.read()

        final_html = template.replace('{{description_text}}', description)
        final_html = final_html.replace('{{image_gallery_html}}', gallery_html)
        final_html = final_html.replace('{{artifact_title}}', clean_title)
        final_html = final_html.replace('{{scroll_class}}', scroll_class)

        # Output index.html exactly where layout.html lives
        index_path = os.path.join(SCRIPT_DIR, 'index.html')
        with open(index_path, 'w') as f:
            f.write(final_html)
        
        print(f"index.html rebuilt for: {command_name}")

        # Write current state file safely
        state_data = {
            "command": command_name,
            "timestamp": time.time()
        }

        state_path = os.path.join(SCRIPT_DIR, 'current_state.json')
        with open(state_path, 'w') as f:
            json.dump(state_data, f)
    
        print(f"Signal sent for {command_name}")

    def parse_tsp_waypoints(self, data):
        selected_commands = data.get('waypoints', data.get('commands'))
        if not isinstance(selected_commands, list):
            raise ValueError("Expected JSON field 'commands' or 'waypoints' to be a list")

        waypoints = []
        for command in selected_commands:
            if isinstance(command, int):
                waypoint = command
            elif isinstance(command, str):
                if command.startswith('artifact_'):
                    waypoint = int(command.rsplit('_', 1)[1]) - 1
                else:
                    waypoint = int(command)
            else:
                raise ValueError(f"Unsupported waypoint value: {command!r}")

            if waypoint < 0:
                raise ValueError(f"Waypoint index must be non-negative: {waypoint}")

            waypoints.append(waypoint)

        return waypoints


# Global server loop runner function (cleanly isolated from Node class scopes)
def start_server(node):
    # Force the local server context directory straight to our root assets folder
    os.chdir(SCRIPT_DIR)
    
    PORT = 8000

    # Define custom request handler class inline to access the 'node' reference directly
    class ROSRequestHandler(http.server.SimpleHTTPRequestHandler):
        def do_POST(self):
            if self.path == '/tour_retrieve':
                content_length = int(self.headers['Content-Length'])
                post_data = self.rfile.read(content_length)
                
                try:
                    data = json.loads(post_data.decode('utf-8'))
                    
                    waypoints = node.parse_tsp_waypoints(data)
                    msg = TspCommand()
                    msg.waypoints = waypoints
                    node.tsp_command_publisher.publish(msg)
                    node.get_logger().info(f"Published TSP command to /tsp_command: {waypoints}")
                    
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.end_headers()
                    response = {
                        "status": "published",
                        "topic": "/tsp_command",
                        "waypoints": waypoints
                    }
                    self.wfile.write(json.dumps(response).encode('utf-8'))
                    
                except Exception as e:
                    node.get_logger().error(f"Failed to parse POST data: {str(e)}")
                    self.send_response(400)
                    self.end_headers()
            else:
                self.send_response(404)
                self.end_headers()

    socketserver.TCPServer.allow_reuse_address = True
    
    with socketserver.TCPServer(("", PORT), ROSRequestHandler) as httpd:
        print(f"Serving at http://localhost:{PORT}")
        
        def open_browser():
            time.sleep(0.5)
            print("Opening browser...")
            webbrowser.open(f"http://localhost:{PORT}/index.html")

        ROSRequestHandler.extensions_map.update({
            '.webp': 'image/webp',
        })
        threading.Thread(target=open_browser).start()
        
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            httpd.shutdown()
            print("\nServer stopped.")


def main(args=None):
    rclpy.init(args=args)
    node = TabletBuilderNode()
    
    # Target our standalone global server handler function
    threading.Thread(target=start_server, args=(node,), daemon=True).start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
