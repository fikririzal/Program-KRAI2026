#!/usr/bin/env python3

import json
import threading
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node

from eira_msgs.msg import WebGuiData


WEB_DIR = Path(__file__).resolve().parent / "web"

INDEX_HTML = WEB_DIR / "index.html"
EMERGENCY_HTML = WEB_DIR / "emergency.html"

ros_node = None


class WebButtonReceiver(Node):
    def __init__(self):
        super().__init__("web_button_receiver")

        self.publisher_ = self.create_publisher(
            WebGuiData,
            "/web_gui_data",
            10
        )

        self.latest_button = [0, 0, 0, 0, 0]
        self.latest_emergency = False
        self.lock = threading.Lock()

        # 10 Hz
        self.timer = self.create_timer(0.1, self.timer_publish)

        self.get_logger().info("Web button receiver aktif")
        self.get_logger().info("Topic: /web_gui_data")
        self.get_logger().info("Rate: 10 Hz")
        self.get_logger().info("Button page: http://IP_ROBOT:8080/")
        self.get_logger().info("Emergency page: http://IP_ROBOT:8080/emergency.html")
        self.get_logger().info(f"WEB_DIR = {WEB_DIR}")

    def update_button(self, button_array):
        with self.lock:
            self.latest_button = button_array

        self.get_logger().info(f"Update btn_tictactoe = {button_array}")

    def update_emergency(self, emergency_value):
        with self.lock:
            self.latest_emergency = bool(emergency_value)

        self.get_logger().info(f"Update emergency = {bool(emergency_value)}")

    def timer_publish(self):
        with self.lock:
            button_array = list(self.latest_button)
            emergency_value = bool(self.latest_emergency)

        msg = WebGuiData()
        msg.stamp = self.get_clock().now().to_msg()
        msg.btn_tictactoe = button_array
        msg.emergency = emergency_value

        self.publisher_.publish(msg)


class WebHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        try:
            if self.path == "/" or self.path == "/index.html":
                self.serve_file(INDEX_HTML)
                return

            if self.path == "/emergency.html":
                self.serve_file(EMERGENCY_HTML)
                return

            self.send_text(404, "404 Not Found")

        except Exception as e:
            self.send_text(500, f"GET error: {e}")

    def do_POST(self):
        try:
            if self.path == "/api/button":
                self.handle_button()
                return

            if self.path == "/api/emergency":
                self.handle_emergency()
                return

            self.send_text(404, "404 Not Found")

        except Exception as e:
            self.send_json({
                "ok": False,
                "error": str(e)
            }, status_code=500)

    def handle_button(self):
        global ros_node

        content_length = int(self.headers.get("Content-Length", 0))
        raw_body = self.rfile.read(content_length).decode("utf-8")
        data = json.loads(raw_body)

        btn_tictactoe = data.get("btn_tictactoe", [0, 0, 0, 0, 0])

        if not isinstance(btn_tictactoe, list):
            raise ValueError("btn_tictactoe harus array/list")

        if len(btn_tictactoe) != 5:
            raise ValueError("btn_tictactoe harus berisi 5 data")

        btn_tictactoe = [int(x) for x in btn_tictactoe]

        for value in btn_tictactoe:
            if value not in [0, 1]:
                raise ValueError("isi btn_tictactoe hanya boleh 0 atau 1")

        if ros_node is not None:
            ros_node.update_button(btn_tictactoe)

        self.send_json({
            "ok": True,
            "btn_tictactoe": btn_tictactoe
        })

    def handle_emergency(self):
        global ros_node

        content_length = int(self.headers.get("Content-Length", 0))
        raw_body = self.rfile.read(content_length).decode("utf-8")
        data = json.loads(raw_body)

        emergency = data.get("emergency", False)

        if not isinstance(emergency, bool):
            raise ValueError("emergency harus bool true/false")

        if ros_node is not None:
            ros_node.update_emergency(emergency)

        self.send_json({
            "ok": True,
            "emergency": emergency
        })

    def serve_file(self, file_path):
        if not file_path.exists():
            self.send_text(
                500,
                f"File HTML tidak ditemukan:\n{file_path}"
            )
            return

        html = file_path.read_bytes()

        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(html)))
        self.end_headers()
        self.wfile.write(html)

    def send_json(self, data, status_code=200):
        response = json.dumps(data).encode("utf-8")

        self.send_response(status_code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(response)))
        self.end_headers()
        self.wfile.write(response)

    def send_text(self, status_code, text):
        response = text.encode("utf-8")

        self.send_response(status_code)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("Content-Length", str(len(response)))
        self.end_headers()
        self.wfile.write(response)

    def log_message(self, format, *args):
        return


def run_web_server():
    host = "0.0.0.0"
    port = 8080

    server = ThreadingHTTPServer((host, port), WebHandler)
    print(f"Web aktif di http://{host}:{port}")
    server.serve_forever()


def main(args=None):
    global ros_node

    rclpy.init(args=args)

    ros_node = WebButtonReceiver()

    web_thread = threading.Thread(
        target=run_web_server,
        daemon=True
    )
    web_thread.start()

    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass

    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()