"""
socket_server.py
----------------
TCP socket server that receives shape requests from the ABB robot,
looks up the latest detection from the vision pipeline, and returns
real-world coordinates and angle.

Protocol:
  Robot  → PC : shape name string  e.g. "Circle"
  PC     → Robot : "x,y,angle"     e.g. "42,87,30"
  Robot  → PC : "quit"             to close connection
"""

import socket
import queue

# Import shared data queue (populated by camera_inference.py)
from camera_inference import data_queue
from configs.config import SERVER_IP, SERVER_PORT


def run_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((SERVER_IP, SERVER_PORT))
    server_socket.listen(1)

    print(f"[Server] Listening on {SERVER_IP}:{SERVER_PORT}")
    client_socket, client_address = server_socket.accept()
    print(f"[Server] Robot connected from {client_address}")

    try:
        while True:
            raw = client_socket.recv(4096).decode("latin-1").strip()
            if not raw:
                continue

            print(f"[Server] Received from robot: '{raw}'")

            if raw.lower() == "quit":
                client_socket.send("quit".encode("UTF-8"))
                break

            # Drain the queue, keep only the latest frame's detections
            latest_objects = []
            while not data_queue.empty():
                latest_objects = data_queue.get()

            # Find best matching object for requested shape
            matches = [
                obj for obj in latest_objects
                if obj["shape"].lower() == raw.lower()
            ]

            if matches:
                best = max(matches, key=lambda o: o["confidence"])
                x = int(round(abs(best["mm_center"][0])))
                y = int(round(abs(best["mm_center"][1])))
                a = int(round(abs(best["angle_deg"])))
                response = f"{x},{y},{a}"
            else:
                response = "Invalid object"

            client_socket.send(response.encode("UTF-8"))
            print(f"[Server] Sent to robot: '{response}'")

    finally:
        client_socket.close()
        server_socket.close()
        print("[Server] Connection closed.")


if __name__ == "__main__":
    run_server()
