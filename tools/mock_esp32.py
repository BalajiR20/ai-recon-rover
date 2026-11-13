"""Standalone mock HTTP server simulating ESP32 DevKit hardware protocol v1."""
from __future__ import annotations

import threading
import time
from typing import Optional
from http.server import BaseHTTPRequestHandler, HTTPServer
import json


class MockEsp32Server:
    def __init__(self, host: str = "127.0.0.1", port: int = 8081, speed_scale: float = 1.0) -> None:
        self.host = host
        self.port = port
        self.speed_scale = speed_scale
        self.server: Optional[HTTPServer] = None
        self.thread: Optional[threading.Thread] = None

        self.boot_id = 1001
        self.active_id = 0
        self.state = "idle"
        self.last_executed_state = "none"
        self.executed_cm = 0.0
        self.executed_deg = 0.0
        self.ultrasonic_cm = 55.0
        self.configured = False

    @property
    def base_url(self) -> str:
        return f"http://{self.host}:{self.port}"

    def start(self) -> None:
        parent = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, format, *args):
                return

            def do_GET(self):
                if self.path == "/status":
                    data = {
                        "boot_id": parent.boot_id,
                        "last_id": parent.active_id,
                        "state": parent.state,
                        "last_executed_state": parent.last_executed_state,
                        "executed_cm": parent.executed_cm,
                        "executed_deg": parent.executed_deg,
                        "ultrasonic_cm": parent.ultrasonic_cm,
                        "rssi": -48,
                        "uptime_ms": 12000,
                        "fw_version": "5.5.0",
                        "connected": True,
                    }
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    self.wfile.write(json.dumps(data).encode("utf-8"))

            def do_POST(self):
                length = int(self.headers.get("Content-Length", 0))
                body = self.rfile.read(length).decode("utf-8") if length > 0 else "{}"
                data = json.loads(body) if body else {}

                if self.path == "/config":
                    parent.configured = True
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    self.wfile.write(b'{"accepted":true}')
                elif self.path == "/move":
                    parent.active_id = data.get("id", 101)
                    val = float(data.get("value", 15.0))
                    mtype = data.get("type", "forward")
                    parent.executed_cm = abs(val) if mtype != "turn" else 0.0
                    parent.executed_deg = val if mtype == "turn" else 0.0
                    parent.state = "idle"
                    parent.last_executed_state = "done"
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    self.wfile.write(b'{"accepted":true,"state":"idle"}')
                elif self.path in ("/stop", "/estop_reset"):
                    parent.state = "idle"
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    self.wfile.write(b'{"accepted":true,"state":"idle"}')
                elif self.path == "/estop":
                    parent.state = "estopped"
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    self.wfile.write(b'{"accepted":true,"state":"estopped"}')
                else:
                    self.send_response(200)
                    self.end_headers()
                    self.wfile.write(b'{"status":"ok"}')

        self.server = HTTPServer((self.host, self.port), Handler)
        self.port = self.server.server_address[1]
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        if self.server:
            self.server.shutdown()
            self.server.server_close()
