"""
config.py
---------
Central configuration for the vision-guided pick & place system.
Edit these values to match your hardware setup before running.
"""

# ── Network ───────────────────────────────────────────────
SERVER_IP   = "192.168.125.201"  # PC IP on the robot network
SERVER_PORT = 5000

# ── Model ─────────────────────────────────────────────────
MODEL_PATH       = "best.pt"     # Path to trained YOLOv11 segmentation weights
CONF_THRESHOLD   = 0.5           # Minimum detection confidence

# ── Camera ────────────────────────────────────────────────
CAMERA_INDEX = 0                 # Basler camera index (0 = first camera)

# ── ArUco Calibration ─────────────────────────────────────
ARUCO_LENGTH = 100               # Physical size of ArUco marker in mm
                                 # Must match the printed marker exactly

# ── Detection ─────────────────────────────────────────────
ROI_BUFFER = 10                  # Pixel buffer around detected bounding box

# ── Debug ─────────────────────────────────────────────────
DEBUG_DIR         = "debug_images"   # Folder to save debug/threshold images
SAVE_DEBUG_IMAGES = True             # Set False to disable debug image saving
