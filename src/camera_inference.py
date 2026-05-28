"""
camera_inference.py
-------------------
Vision pipeline for the robot pick & place system.

Responsibilities:
  1. Connect to Basler camera via pypylon
  2. Calibrate coordinate system using an ArUco marker
  3. Detect objects (circle, square, triangle) with YOLOv11 segmentation
  4. Estimate pick angle per shape using contour analysis
  5. Convert pixel coordinates to real-world mm using homography
  6. Push detected objects to a shared queue for the socket server

Controls (OpenCV window):
  c  — recalibrate ArUco marker
  q  — quit
"""

import cv2
import numpy as np
import math
import os
import queue
import threading

from ultralytics import YOLO
from pypylon import pylon

from configs.config import (
    MODEL_PATH, CONF_THRESHOLD, CAMERA_INDEX,
    ARUCO_LENGTH, ROI_BUFFER, DEBUG_DIR, SAVE_DEBUG_IMAGES
)

# Shared queue: camera thread → socket server thread
data_queue = queue.Queue()

if SAVE_DEBUG_IMAGES:
    os.makedirs(DEBUG_DIR, exist_ok=True)


class RotBox:
    """Stores the four corner coordinates of a detected bounding box."""
    def __init__(self, cls, score, x1, y1, x2, y2, x3, y3, x4, y4):
        self.cls = cls
        self.score = score
        self.x1, self.y1 = x1, y1
        self.x2, self.y2 = x2, y2
        self.x3, self.y3 = x3, y3
        self.x4, self.y4 = x4, y4


class CameraInference:
    """
    Main vision pipeline class.

    Connects to a Basler camera, calibrates using an ArUco marker,
    runs YOLOv11 segmentation, computes real-world coordinates and
    pick angles, and publishes results to `data_queue`.
    """

    def __init__(self, model_path: str = MODEL_PATH, camera_index: int = CAMERA_INDEX):
        self.model = YOLO(model_path, task='segment')
        self.camera = None
        self.camera_index = camera_index
        self.running = False

        # ArUco setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        self.parameters = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        self.aruco_length = ARUCO_LENGTH

        # Calibration state
        self.pixel_mm_ratio = None
        self.aruco_origin_pixel = None
        self.scale_point1_pixel = None
        self.scale_point2_pixel = None
        self.y_axis_point_pixel = None
        self.aruco_rotation_angle = 0
        self.homography_matrix = None
        self.is_calibrated = False

    # ──────────────────────────────────────────────
    # Camera
    # ──────────────────────────────────────────────

    def connect_camera(self):
        """Connect to the first available Basler camera."""
        tl_factory = pylon.TlFactory.GetInstance()
        devices = tl_factory.EnumerateDevices()
        if not devices:
            raise RuntimeError("No Basler camera found. Check USB/GigE connection.")
        self.camera = pylon.InstantCamera(
            tl_factory.CreateDevice(devices[self.camera_index])
        )
        self.camera.Open()
        print("[Camera] Connected.")

    # ──────────────────────────────────────────────
    # ArUco Calibration
    # ──────────────────────────────────────────────

    def calibrate_aruco_transform(self, img: np.ndarray) -> tuple:
        """
        Detect ArUco marker and compute homography for pixel→mm mapping.

        Returns:
            (success: bool, marker_corners: np.ndarray | None)
        """
        corners, ids, _ = self.aruco_detector.detectMarkers(img)
        if corners and ids is not None and len(ids) > 0:
            marker_corners = corners[0][0].reshape(4, 2).astype(np.float32)
            self.aruco_origin_pixel  = marker_corners[0]
            self.scale_point1_pixel  = marker_corners[0]
            self.scale_point2_pixel  = marker_corners[3]
            self.y_axis_point_pixel  = marker_corners[1]

            pixel_dist = np.linalg.norm(self.scale_point2_pixel - self.scale_point1_pixel)
            self.pixel_mm_ratio = self.aruco_length / pixel_dist

            x_vec = self.scale_point2_pixel - self.scale_point1_pixel
            self.aruco_rotation_angle = math.degrees(math.atan2(x_vec[1], x_vec[0]))

            obj_points = np.array(
                [[0, 0], [0, self.aruco_length],
                 [self.aruco_length, self.aruco_length], [self.aruco_length, 0]],
                dtype=np.float32
            )
            self.homography_matrix, _ = cv2.findHomography(marker_corners, obj_points)
            self.is_calibrated = True
            print(f"[Calibration] Complete. Rotation: {self.aruco_rotation_angle:.2f}°")
            return True, marker_corners
        else:
            self.is_calibrated = False
            cv2.putText(img, "Place ArUco Marker & press C",
                        (50, img.shape[0] // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            return False, None

    def draw_aruco_axes(self, img: np.ndarray, marker_corners):
        """Overlay ArUco coordinate axes on the image."""
        if marker_corners is None:
            return
        origin = self.aruco_origin_pixel.astype(int)
        x_end  = self.scale_point2_pixel.astype(int)
        y_end  = self.y_axis_point_pixel.astype(int)

        cv2.circle(img, tuple(origin), 8, (0, 0, 255), -1)
        cv2.putText(img, "ORIGIN (0,0)", (origin[0] + 10, origin[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.line(img, tuple(origin), tuple(x_end), (0, 255, 0), 3)
        cv2.putText(img, "X (100mm)", (x_end[0] + 10, x_end[1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.line(img, tuple(origin), tuple(y_end), (255, 0, 0), 3)
        cv2.putText(img, "Y-AXIS", (y_end[0] + 10, y_end[1] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        cv2.polylines(img, [marker_corners.astype(np.int32)], True, (255, 255, 255), 2)

    def pixel_to_mm(self, pixel_coords: list) -> np.ndarray | None:
        """Convert pixel coordinates to mm using the calibrated homography."""
        if not self.is_calibrated or self.homography_matrix is None:
            return None
        pt = np.array([pixel_coords[0], pixel_coords[1], 1], dtype=np.float32)
        mm_h = self.homography_matrix @ pt
        mm = mm_h[:2] / mm_h[2]
        origin_h = self.homography_matrix @ np.array(
            [self.aruco_origin_pixel[0], self.aruco_origin_pixel[1], 1]
        )
        origin_mm = origin_h[:2] / origin_h[2]
        return mm - origin_mm

    # ──────────────────────────────────────────────
    # Angle Estimation
    # ──────────────────────────────────────────────

    def _estimate_angle(self, rotbox: RotBox, img: np.ndarray, class_name: str) -> tuple:
        """
        Extract pick centre and angle from a detected object's bounding box.

        Returns:
            (cx, cy, w, h, angle) in pixel space, or (None,)*5 on failure.
        """
        h0, w0 = img.shape[:2]
        corners = np.array([
            [rotbox.x1, rotbox.y1], [rotbox.x2, rotbox.y2],
            [rotbox.x3, rotbox.y3], [rotbox.x4, rotbox.y4]
        ], dtype=np.float32)

        x_min = max(int(corners[:, 0].min()) - ROI_BUFFER, 0)
        x_max = min(int(corners[:, 0].max()) + ROI_BUFFER, w0 - 1)
        y_min = max(int(corners[:, 1].min()) - ROI_BUFFER, 0)
        y_max = min(int(corners[:, 1].max()) + ROI_BUFFER, h0 - 1)

        roi = img[y_min:y_max, x_min:x_max].copy()
        if roi.size == 0:
            return None, None, None, None, None

        # Threshold to isolate object
        gray  = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur  = cv2.GaussianBlur(gray, (5, 5), 0)
        thresh = cv2.adaptiveThreshold(
            blur, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 21, 4
        )
        thresh = cv2.GaussianBlur(thresh, (3, 3), 0)

        if SAVE_DEBUG_IMAGES:
            cv2.imwrite(
                os.path.join(DEBUG_DIR, f"thresh_{class_name}_{int(cv2.getTickCount())}.png"),
                thresh
            )

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, None, None, None, None

        largest   = max(contours, key=cv2.contourArea)
        min_rect  = cv2.minAreaRect(largest)
        (cx_rel, cy_rel), (w, h), rect_angle = min_rect
        cx = cx_rel + x_min
        cy = cy_rel + y_min

        # Draw rotated bounding box
        box_pts = cv2.boxPoints(min_rect) + np.array([x_min, y_min])
        cv2.polylines(img, [box_pts.astype(np.int32)], True, (255, 0, 255), 2)

        # ── Angle logic per shape ──────────────────
        if class_name.lower() == "circle":
            angle = 0.0

        elif class_name.lower() == "triangle" and self.is_calibrated:
            angle = self._triangle_angle(largest, img, x_min, y_min, class_name)

        else:
            # Square / unknown: use minAreaRect, normalise to 0–90°
            angle = min(rect_angle, 180 - rect_angle) + 55

        return cx, cy, w, h, angle

    def _triangle_angle(self, contour, img, x_min, y_min, class_name) -> float:
        """Find the triangle edge nearest to the ArUco y-axis and return its angle."""
        epsilon = 0.03 * cv2.arcLength(contour, True)
        approx  = cv2.approxPolyDP(contour, epsilon, True)

        if len(approx) != 3:
            print(f"[Triangle] Expected 3 vertices, got {len(approx)}. Defaulting to 0°.")
            return 0.0

        verts = approx.reshape(-1, 2).astype(np.float32)
        a = np.linalg.norm(verts[1] - verts[0])
        b = np.linalg.norm(verts[2] - verts[1])
        c = np.linalg.norm(verts[0] - verts[2])
        if not (a + b > c and b + c > a and c + a > b):
            return 0.0

        y1_pt = self.aruco_origin_pixel
        y2_pt = self.y_axis_point_pixel
        y_vec = y2_pt - y1_pt
        y_len = np.linalg.norm(y_vec)

        edges = []
        for i in range(3):
            pt1 = verts[i]
            pt2 = verts[(i + 1) % 3]
            mid = (pt1 + pt2) / 2
            x0, y0 = mid
            x1, y1 = y1_pt
            x2, y2 = y2_pt
            dist = abs((x2-x1)*(y1-y0) - (x1-x0)*(y2-y1)) / max(y_len, 1e-6)
            ev   = pt2 - pt1
            el   = np.linalg.norm(ev)
            dot  = np.clip(np.dot(ev, y_vec) / max(el * y_len, 1e-6), -1, 1)
            ang  = math.degrees(math.acos(dot))
            ang  = min(ang, 180 - ang)
            edges.append({'pt1': pt1, 'pt2': pt2, 'dist': dist, 'angle': ang})

        edges.sort(key=lambda e: e['dist'])
        best = edges[0]

        # Draw selected edge on debug image
        p1 = (best['pt1'] + np.array([x_min, y_min])).astype(np.int32)
        p2 = (best['pt2'] + np.array([x_min, y_min])).astype(np.int32)
        cv2.line(img, tuple(p1), tuple(p2), (255, 0, 0), 3)

        angle = best['angle'] + 30  # empirical offset for gripper alignment
        print(f"[Triangle] Selected edge {p1}→{p2}, dist={best['dist']:.1f}, angle={angle:.1f}°")
        return angle

    # ──────────────────────────────────────────────
    # Detection
    # ──────────────────────────────────────────────

    def detect_objects(self, img: np.ndarray) -> list:
        """Run YOLO on a frame and return list of detected object dicts."""
        results = self.model(img, verbose=False)
        detected = []

        for result in results:
            if result.boxes is None:
                continue
            for i in range(len(result.boxes.cls)):
                score = float(result.boxes.conf[i])
                if score < CONF_THRESHOLD:
                    continue

                x_min, y_min, x_max, y_max = result.boxes.xyxy[i].flatten().tolist()
                cls_id     = int(result.boxes.cls[i])
                class_name = result.names[cls_id]

                rotbox = RotBox(
                    cls=cls_id, score=score,
                    x1=x_min, y1=y_min,
                    x2=x_max, y2=y_min,
                    x3=x_max, y3=y_max,
                    x4=x_min, y4=y_max
                )
                cx, cy, w, h, angle = self._estimate_angle(rotbox, img, class_name)
                if cx is None:
                    continue

                mm = self.pixel_to_mm([cx, cy])
                if mm is None:
                    continue

                detected.append({
                    "shape":       class_name,
                    "pixel_center": [float(cx), float(cy)],
                    "mm_center":    [float(mm[0]), float(mm[1])],
                    "angle_deg":    float(angle),
                    "confidence":   float(score),
                    "mask_bbox":    [x_min, y_min, x_max, y_max],
                    "size_px":      [float(w), float(h)]
                })
                print(f"[Detection] {class_name}: "
                      f"({mm[0]:.1f}, {mm[1]:.1f}) mm  "
                      f"angle={angle:.1f}°  conf={score:.2f}")

        return detected

    # ──────────────────────────────────────────────
    # Main Loop
    # ──────────────────────────────────────────────

    def run(self):
        self.connect_camera()
        self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        converter = pylon.ImageFormatConverter()
        converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        self.running = True

        print("[System] Press 'c' to calibrate ArUco marker, 'q' to quit.")

        while self.running:
            grab = self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            if not grab.GrabSucceeded():
                grab.Release()
                continue

            raw  = grab.GetArray().copy()
            img  = pylon.ImageFormatConverter().Convert(grab).GetArray().copy() \
                   if hasattr(grab, 'GetArray') else raw
            grab.Release()

            key = cv2.waitKey(1) & 0xFF

            if not self.is_calibrated or key == ord('c'):
                _, marker_corners = self.calibrate_aruco_transform(img)
            else:
                corners, ids, _ = self.aruco_detector.detectMarkers(img)
                marker_corners = corners[0][0].reshape(4, 2) if corners else None

            if marker_corners is not None:
                cv2.aruco.drawDetectedMarkers(img, [marker_corners.reshape(1, 4, 2)])
            self.draw_aruco_axes(img, marker_corners)

            if self.is_calibrated:
                objects = self.detect_objects(img)
                if objects:
                    data_queue.put(objects)
                for obj in objects:
                    px, py = map(int, obj['pixel_center'])
                    mx, my = obj['mm_center']
                    cv2.circle(img, (px, py), 6, (0, 255, 0), -1)
                    cv2.putText(
                        img,
                        f"{obj['shape']} ({mx:.1f},{my:.1f})mm  {obj['angle_deg']:.1f}°",
                        (px, py - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2
                    )

            status_text  = "CALIBRATED" if self.is_calibrated else "PRESS C TO CALIBRATE"
            status_color = (0, 255, 0) if self.is_calibrated else (0, 0, 255)
            cv2.putText(img, status_text, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
            cv2.imshow("Vision System", img)

            if key == ord('q'):
                self.running = False

        self.camera.StopGrabbing()
        self.camera.Close()
        cv2.destroyAllWindows()

    def stop(self):
        self.running = False


# ──────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────

if __name__ == "__main__":
    import threading
    from src.socket_server import run_server

    threading.Thread(target=run_server, daemon=True).start()
    CameraInference().run()
