# main.py
# Entry point — run this file to start the system.
# Replicates the exact __main__ block from main_v2.py:
#   - inference_thread: starts camera + YOLO detection loop
#   - server_thread:    starts TCP socket server for robot communication
# Both threads share data_queue (defined in src/camera_inference.py).
#
# Usage:
#   python main.py
#
# Before running:
#   1. Update MODEL_PATH in configs/config.py
#   2. Update SERVER_IP in configs/config.py to your PC's IP
#   3. Connect Basler camera
#   4. Press 'c' in the OpenCV window to calibrate ArUco marker
#   5. Press 'q' to quit

import threading
import sys
import os

from configs.config import MODEL_PATH
from src.camera_inference import CameraInference
from src.socket_server import run_server


def inference_thread():
    camera_inf = CameraInference(MODEL_PATH)
    camera_inf.run()


def server_thread():
    run_server()


if __name__ == "__main__":
    threading.Thread(target=inference_thread, daemon=True).start()
    threading.Thread(target=server_thread, daemon=True).start()
    try:
        while True:
            threading.Event().wait(0.1)
    except KeyboardInterrupt:
        print("Shutting down...")
