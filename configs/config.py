# config.py
# Central configuration — mirrors the constants defined at the top of main_v2.py
# Edit these to match your hardware setup before running.

# ── Network ───────────────────────────────────────────────────────────────────
SERVER_IP   = "192.168.125.201"  # PC IP on the robot network subnet
SERVER_PORT = 5000

# ── Model ─────────────────────────────────────────────────────────────────────
MODEL_PATH = "best.pt"           # Path to trained YOLOv11 segmentation weights
                                 # Update this to your actual path before running

# ── Detection ─────────────────────────────────────────────────────────────────
CONF_THRESHOLD = 0.5             # Minimum detection confidence (original value)
BUFFER         = 10              # Pixel buffer around bounding box ROI

# ── ArUco ─────────────────────────────────────────────────────────────────────
ARUCO_LENGTH = 100               # Physical size of ArUco marker in mm
                                 # Must match the printed marker exactly

# ── Debug ─────────────────────────────────────────────────────────────────────
DEBUG_DIR = "debug_images"       # Folder for threshold/contour debug images
