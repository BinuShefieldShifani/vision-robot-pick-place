# 🤖 Vision-Guided Robot Pick & Place System

> End-to-end robotic automation system combining **YOLOv11 instance segmentation**, **ArUco-based spatial calibration**, and **ABB robot control** for autonomous object pick & place — built as a university final project.

![Python](https://img.shields.io/badge/Python-3.10+-blue?logo=python&logoColor=white)
![YOLOv11](https://img.shields.io/badge/YOLOv11-Segmentation-orange)
![ABB Robot](https://img.shields.io/badge/ABB-RAPID-red)
![OpenCV](https://img.shields.io/badge/OpenCV-ArUco-green)
![Status](https://img.shields.io/badge/Status-Complete-brightgreen)

---

## 📌 Overview

This project demonstrates a fully integrated **vision-guided robotic pick & place pipeline**. A Basler camera detects geometric objects (circle, square, triangle) on a conveyor using a custom-trained YOLOv11 segmentation model. The system computes real-world coordinates and orientation angles using ArUco marker calibration, then communicates them to an ABB robot via a TCP/IP socket — which autonomously picks and places each object with the correct orientation.

---

## 🎯 Key Capabilities

- **Real-time object detection** — YOLOv11 segmentation on live Basler camera feed
- **ArUco spatial calibration** — pixel-to-mm coordinate transformation using homography
- **Shape-specific angle estimation** — custom logic per object type for accurate gripper orientation
- **TCP/IP robot communication** — Python server ↔ ABB RAPID client
- **Autonomous pick & place** — vacuum gripper with orientation-corrected placement

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    VISION SYSTEM (PC)                   │
│                                                         │
│  Basler Camera (pypylon)                                │
│       │                                                 │
│       ▼                                                 │
│  ArUco Calibration ──► Pixel → MM Transform             │
│       │                                                 │
│       ▼                                                 │
│  YOLOv11 Segmentation                                   │
│  (circle / square / triangle)                           │
│       │                                                 │
│       ▼                                                 │
│  Angle Estimation (shape-specific)                      │
│       │                                                 │
│       ▼                                                 │
│  TCP Socket Server  ◄──────────────────────────────┐   │
│  → sends: x_mm, y_mm, angle                        │   │
└────────────────────────────────────────────────────┼───┘
                                                     │
                         TCP/IP (Socket)             │
                                                     │
┌────────────────────────────────────────────────────┴───┐
│                   ABB ROBOT (RAPID)                     │
│                                                         │
│  Operator selects shape on teach pendant                │
│       │                                                 │
│       ▼                                                 │
│  Send shape name → PC server                            │
│  Receive x, y, angle ← PC server                       │
│       │                                                 │
│       ▼                                                 │
│  ParseCoordinates()                                     │
│       │                                                 │
│       ▼                                                 │
│  PickPlace()                                            │
│  • MoveL to pick position (with offset)                 │
│  • Activate vacuum gripper (doValve1)                   │
│  • MoveL home                                           │
│  • Apply rotation to place target                       │
│  • MoveL to place position                              │
│  • Release gripper                                      │
└─────────────────────────────────────────────────────────┘
```

---

## 🧠 Technical Details

### Vision Pipeline (`src/camera_inference.py`)

**Camera:** Basler industrial camera via `pypylon`

**Calibration:**
- ArUco marker (100mm, `DICT_5X5_100`) placed in the scene
- Homography matrix computed from 4 marker corners
- Any pixel coordinate transforms to real-world mm instantly

**Detection:**
- YOLOv11 segmentation model trained on 500 images annotated in CVAT (YOLO format)
- Classes: `circle`, `square`, `triangle`
- Confidence threshold: 0.5

**Angle Estimation (shape-specific logic):**

| Shape | Method |
|---|---|
| Circle | Always 0° (rotationally symmetric) |
| Square | `minAreaRect` angle from largest contour |
| Triangle | Finds the triangle edge nearest to ArUco y-axis, computes angle relative to it |

**Coordinate output per object:**
```
shape, pixel_center, mm_center, angle_deg, confidence, mask_bbox
```

### Robot Communication (`src/socket_server.py`)

- Python runs a TCP socket server on port 5000
- ABB robot connects as client, sends shape name (e.g. `"Circle"`)
- Server returns `"x,y,angle"` as a comma-separated string
- Robot parses and executes pick & place

### Robot Program (`robot/MainModule.mod`)

Written in **ABB RAPID**:
- Operator selects object (1=Circle, 2=Square, 3=Triangle) on teach pendant
- Robot queries PC, receives coordinates
- `PickPlace()` procedure handles full motion sequence with vacuum gripper
- Placement orientation is corrected using the received angle (Euler ZYX)

---

## 📁 Repository Structure

```
vision-robot-pick-place/
├── src/
│   ├── camera_inference.py     # Full vision pipeline (camera, ArUco, YOLO, angle)
│   └── socket_server.py        # TCP socket server for robot communication
├── robot/
│   └── MainModule.mod          # ABB RAPID robot program
├── configs/
│   └── config.py               # IP, port, thresholds, paths
├── docs/
│   └── system_overview.md      # Hardware setup and wiring notes
├── results/
│   └── demo/                   # Demo images/videos (coming soon)
├── requirements.txt
└── README.md
```

---

## 🚀 Getting Started

### 1. Install Dependencies

```bash
pip install -r requirements.txt
```

> **Note:** `pypylon` requires the [Basler Pylon SDK](https://www.baslerweb.com/en/downloads/software-downloads/) to be installed first.

### 2. Configure the System

Edit `configs/config.py`:

```python
SERVER_IP = "192.168.125.201"   # Your PC's IP on the robot network
SERVER_PORT = 5000
MODEL_PATH = "best.pt"          # Path to your trained YOLOv11 model
CONF_THRESHOLD = 0.5
ARUCO_LENGTH = 100              # mm — physical size of your ArUco marker
```

### 3. Print the ArUco Marker

Generate and print a **100mm ArUco marker** from `DICT_5X5_100`, ID 0. Place it flat in the camera's field of view.

### 4. Run the System

```bash
python src/camera_inference.py
```

- Press **`c`** to calibrate (once ArUco marker is visible)
- Press **`q`** to quit

In a second terminal:

```bash
python src/socket_server.py
```

### 5. Load Robot Program

Upload `robot/MainModule.mod` to the ABB robot controller via RobotStudio or USB. Adjust `wobj1`, `wobj2`, and place targets (`pcircle`, `psquare`, `ptriangle`) to match your physical setup.

---

## 📦 Dataset

- **500 images** of circles, squares, and triangles on a conveyor surface
- Annotated using **CVAT** and exported in **YOLO format**
- Dataset available on request — open an issue with your use case

---

## 🛠️ Tech Stack

| Component | Technology |
|---|---|
| Object Detection | YOLOv11 (Ultralytics) — segmentation |
| Camera | Basler industrial camera (pypylon) |
| Spatial Calibration | ArUco markers + OpenCV homography |
| Angle Estimation | OpenCV contour analysis, minAreaRect |
| Robot | ABB (RAPID programming language) |
| Communication | TCP/IP socket (Python ↔ RAPID) |
| Annotation Tool | CVAT |
| Training Environment | Local GPU / Google Colab |

---

## ⚙️ Hardware Requirements

- Basler industrial camera (GigE or USB3)
- ABB robot with vacuum gripper (doValve1 digital output)
- PC on the same network as the robot controller (192.168.125.x subnet)
- Printed ArUco marker (100mm × 100mm, flat surface)

---

## 🔭 Future Work

- [ ] Multi-object handling (pick sequence prioritisation)
- [ ] Replace fixed place targets with dynamic placement zones
- [ ] Add ROS2 integration for broader robot compatibility
- [ ] Export model to ONNX for edge deployment on robot controller
- [ ] Web dashboard for live monitoring and logging

---

## 👤 Author

**Binu Shefield Shifani**

Software Engineer (5 years, Cognizant Technology Solutions)
MS AI & Automation · University West, Trollhättan, Sweden
[GitHub](https://github.com/BinuShefieldShifani)

---

## 📄 License

MIT License — code is free to use for research and educational purposes.
The trained model weights (`best.pt`) are included for reproducibility.
