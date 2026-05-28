# src/socket_server.py
# TCP socket server — exact logic from server_thread() in main_v2.py.
# Imports data_queue from camera_inference (same module that populates it).

import socket
import os
import sys

# Allow imports from repo root (for configs/)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from configs.config import SERVER_IP, SERVER_PORT

# Import shared queue from camera_inference — this is the same object
# that CameraInference.run() puts detected objects into.
from src.camera_inference import data_queue


def run_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((SERVER_IP, SERVER_PORT))
    server_socket.listen(1)
    (client_socket, client_ip) = server_socket.accept()
    while True:
        client_message = client_socket.recv(4096).decode("latin-1").strip()
        if not client_message:
            continue
        print(f"Received from robot: {client_message}")
        if client_message.lower() == "":
            server_message = "quit"
        else:
            latest_objects = []
            while not data_queue.empty():
                latest_objects = data_queue.get()
            matching_objs = [obj for obj in latest_objects if obj["shape"].lower() == client_message.lower()]
            if matching_objs:
                best_obj = max(matching_objs, key=lambda o: o["confidence"])
                mm_coords = best_obj["mm_center"]
                x_val = int(round(abs(mm_coords[0])))
                y_val = int(round(abs(mm_coords[1])))
                a_val = int(round(abs(best_obj["angle_deg"])))
                server_message = f"{x_val},{y_val},{a_val}"
            else:
                server_message = "Invalid object"
        client_socket.send(server_message.encode("UTF-8"))
        print("Sent to robot: ", server_message)
        if server_message == "quit":
            break
    client_socket.close()
    server_socket.close()
