# src/camera_inference.py
# Vision pipeline — exact logic from main_v2.py.
# Owns the shared data_queue that socket_server.py reads from.

import cv2
import numpy as np
from ultralytics import YOLO
from pypylon import pylon
import queue
import math
import os
import sys

# Allow imports from repo root (for configs/)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from configs.config import CONF_THRESHOLD, BUFFER, DEBUG_DIR, MODEL_PATH, ARUCO_LENGTH

# ── Shared queue ──────────────────────────────────────────────────────────────
# Owned here. socket_server.py imports this directly.
data_queue = queue.Queue()

# Ensure debug directory exists
os.makedirs(DEBUG_DIR, exist_ok=True)


class RotBox:
    def __init__(self, cls, score, x1, y1, x2, y2, x3, y3, x4, y4):
        self.cls = cls
        self.score = score
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.x3 = x3
        self.y3 = y3
        self.x4 = x4
        self.y4 = y4


class CameraInference:
    def __init__(self, model_path, camera_index=0):
        self.model = YOLO(model_path, task='segment')
        self.camera = None
        self.camera_index = camera_index
        self.running = False
        # ArUco setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        self.parameters = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        self.aruco_length = ARUCO_LENGTH  # mm known size of ArUco marker
        # Coordinate transformation
        self.pixel_mm_ratio = None
        self.aruco_origin_pixel = None
        self.scale_point1_pixel = None
        self.scale_point2_pixel = None
        self.y_axis_point_pixel = None
        self.aruco_rotation_angle = 0
        self.homography_matrix = None
        self.is_calibrated = False

    def connect_camera(self):
        try:
            tl_factory = pylon.TlFactory.GetInstance()
            devices = tl_factory.EnumerateDevices()
            if not devices:
                raise Exception("No Basler camera found.")
            self.camera = pylon.InstantCamera(tl_factory.CreateDevice(devices[self.camera_index]))
            self.camera.Open()
            print("Camera connected.")
        except Exception as e:
            print(f"Camera connection failed: {e}")
            exit(1)

    def calibrate_aruco_transform(self, img):
        corners, ids, _ = self.aruco_detector.detectMarkers(img)
        if corners and ids is not None and len(ids) > 0:
            marker_corners = corners[0][0].reshape(4, 2).astype(np.float32)
            self.aruco_origin_pixel = marker_corners[0]
            self.scale_point1_pixel = marker_corners[0]
            self.scale_point2_pixel = marker_corners[3]
            self.y_axis_point_pixel = marker_corners[1]
            pixel_distance = np.linalg.norm(self.scale_point2_pixel - self.scale_point1_pixel)
            self.pixel_mm_ratio = self.aruco_length / pixel_distance
            x_axis_vector = self.scale_point2_pixel - self.scale_point1_pixel
            self.aruco_rotation_angle = math.degrees(math.atan2(x_axis_vector[1], x_axis_vector[0]))
            obj_points = np.array([[0, 0], [0, self.aruco_length],
                                   [self.aruco_length, self.aruco_length], [self.aruco_length, 0]], dtype=np.float32)
            self.homography_matrix, _ = cv2.findHomography(marker_corners, obj_points)
            self.is_calibrated = True
            print(f"Calibration complete. Rotation: {self.aruco_rotation_angle:.2f}°")
            print(f"ArUco Y-axis: {self.aruco_origin_pixel} to {self.y_axis_point_pixel}")
            return True, marker_corners
        else:
            print("No ArUco marker detected")
            self.is_calibrated = False
            cv2.putText(img, "Place ArUco Marker", (50, img.shape[0] // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            return False, None

    def draw_aruco_axes(self, img, marker_corners):
        if marker_corners is None:
            return
        origin = self.aruco_origin_pixel.astype(int)
        x_end = self.scale_point2_pixel.astype(int)
        y_end = self.y_axis_point_pixel.astype(int)
        cv2.circle(img, tuple(origin), 8, (0, 0, 255), -1)
        cv2.putText(img, "ORIGIN (0,0)", (origin[0] + 10, origin[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.line(img, tuple(origin), tuple(x_end), (0, 255, 0), 3)
        cv2.putText(img, "X-AXIS (100mm)", (x_end[0] + 10, x_end[1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.line(img, tuple(origin), tuple(y_end), (255, 0, 0), 3)
        cv2.putText(img, "Y-AXIS", (y_end[0] + 10, y_end[1] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        cv2.polylines(img, [marker_corners.astype(np.int32)], True, (255, 255, 255), 2)

    def pixel_to_mm_transform(self, pixel_coords):
        if not self.is_calibrated or self.homography_matrix is None:
            return None
        homo_coords = np.array([pixel_coords[0], pixel_coords[1], 1], dtype=np.float32)
        mm_coords_homo = self.homography_matrix @ homo_coords
        mm_coords = mm_coords_homo[:2] / mm_coords_homo[2]
        origin_homo = self.homography_matrix @ np.array([self.aruco_origin_pixel[0], self.aruco_origin_pixel[1], 1])
        origin_mm = origin_homo[:2] / origin_homo[2]
        mm_coords -= origin_mm
        return mm_coords

    def process_rotbox(self, rotbox, img, class_name):
        h0, w0 = img.shape[:2]
        scale_x, scale_y = 1.0, 1.0
        corners = np.array([
            [rotbox.x1 * scale_x, rotbox.y1 * scale_y],
            [rotbox.x2 * scale_x, rotbox.y2 * scale_y],
            [rotbox.x3 * scale_x, rotbox.y3 * scale_y],
            [rotbox.x4 * scale_x, rotbox.y4 * scale_y]
        ], dtype=np.float32)
        x_min = max(int(np.min(corners[:, 0]) - BUFFER), 0)
        x_max = min(int(np.max(corners[:, 0]) + BUFFER), w0 - 1)
        y_min = max(int(np.min(corners[:, 1]) - BUFFER), 0)
        y_max = min(int(np.max(corners[:, 1]) + BUFFER), h0 - 1)
        roi = img[y_min:y_max, x_min:x_max].copy()
        if roi.size == 0:
            return None, None, None, None, None
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        thresh = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 21, 4)
        thresh = cv2.GaussianBlur(thresh, (3, 3), 0)
        cv2.imwrite(os.path.join(DEBUG_DIR, f"thresh_{class_name}_{int(cv2.getTickCount())}.png"), thresh)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, None, None, None, None
        largest = max(contours, key=cv2.contourArea)
        min_rect = cv2.minAreaRect(largest)
        (cx_rel, cy_rel), (w, h), rect_angle = min_rect
        cx = cx_rel + x_min
        cy = cy_rel + y_min
        box_points = cv2.boxPoints(min_rect)
        box_points = box_points + np.array([x_min, y_min])
        box_points = box_points.astype(np.int32)
        cv2.polylines(img, [box_points], True, (255, 0, 255), 2)
        if class_name.lower() == "circle":
            angle = 0.0
        elif class_name.lower() == "triangle" and self.is_calibrated:
            perimeter = cv2.arcLength(largest, True)
            epsilon = 0.03 * perimeter
            approx = cv2.approxPolyDP(largest, epsilon, True)
            debug_img = roi.copy()
            cv2.drawContours(debug_img, [approx], -1, (0, 255, 0), 2)
            cv2.imwrite(os.path.join(DEBUG_DIR, f"contour_{class_name}_{int(cv2.getTickCount())}.png"), debug_img)
            if len(approx) == 3:
                vertices = approx.reshape(-1, 2).astype(np.float32)
                a = np.linalg.norm(vertices[1] - vertices[0])
                b = np.linalg.norm(vertices[2] - vertices[1])
                c = np.linalg.norm(vertices[0] - vertices[2])
                if (a + b > c) and (b + c > a) and (c + a > b):
                    edges = []
                    y_axis_p1 = self.aruco_origin_pixel
                    y_axis_p2 = self.y_axis_point_pixel
                    for i in range(3):
                        pt1 = vertices[i]
                        pt2 = vertices[(i + 1) % 3]
                        edge_vec = pt2 - pt1
                        edge_length = np.linalg.norm(edge_vec)
                        midpoint = (pt1 + pt2) / 2
                        x0, y0 = midpoint
                        x1, y1 = y_axis_p1
                        x2, y2 = y_axis_p2
                        numerator = abs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1))
                        denominator = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
                        distance = numerator / denominator if denominator > 0 else float('inf')
                        y_axis_vec = y_axis_p2 - y_axis_p1
                        y_axis_length = np.linalg.norm(y_axis_vec)
                        angle_rad = 0.0
                        if edge_length > 0 and y_axis_length > 0:
                            dot = np.dot(edge_vec, y_axis_vec) / (edge_length * y_axis_length)
                            dot = np.clip(dot, -1.0, 1.0)
                            angle_rad = math.acos(dot)
                            edge_angle = math.degrees(angle_rad)
                            edge_angle = min(edge_angle, 180 - edge_angle)
                        else:
                            edge_angle = 0.0
                        edges.append({
                            'pt1': pt1,
                            'pt2': pt2,
                            'length': edge_length,
                            'distance': distance,
                            'angle': edge_angle
                        })
                    edges.sort(key=lambda x: x['distance'])
                    print(f"Triangle - All edges:")
                    for i, edge in enumerate(edges):
                        pt1_img = (edge['pt1'] + np.array([x_min, y_min])).astype(np.int32)
                        pt2_img = (edge['pt2'] + np.array([x_min, y_min])).astype(np.int32)
                        print(f"Edge {i+1}: {pt1_img} to {pt2_img}, length: {edge['length']:.3f}, "
                              f"distance to y-axis: {edge['distance']:.3f}, angle: {edge['angle']:.1f}°")
                    cv2.line(img, tuple((edges[0]['pt1'] + np.array([x_min, y_min])).astype(np.int32)),
                             tuple((edges[0]['pt2'] + np.array([x_min, y_min])).astype(np.int32)), (255, 0, 0), 3)
                    cv2.line(img, tuple((edges[1]['pt1'] + np.array([x_min, y_min])).astype(np.int32)),
                             tuple((edges[1]['pt2'] + np.array([x_min, y_min])).astype(np.int32)), (0, 255, 0), 2)
                    cv2.line(img, tuple((edges[2]['pt1'] + np.array([x_min, y_min])).astype(np.int32)),
                             tuple((edges[2]['pt2'] + np.array([x_min, y_min])).astype(np.int32)), (0, 0, 255), 1)
                    best_edge = edges[0]
                    angle = best_edge['angle']
                    angle = angle + 30
                    pt1_img = (best_edge['pt1'] + np.array([x_min, y_min])).astype(np.int32)
                    pt2_img = (best_edge['pt2'] + np.array([x_min, y_min])).astype(np.int32)
                    print(f"Triangle - Selected edge (nearest to y-axis): {pt1_img} to {pt2_img}, "
                          f"length: {best_edge['length']:.3f}, distance: {best_edge['distance']:.3f}, angle: {angle:.1f}°")
                else:
                    angle = 0.0
                    print(f"Triangle - Invalid triangle (degenerate), setting angle to 0.0°")
            else:
                angle = 0.0
                print(f"Triangle - Contour is not a triangle (vertices: {len(approx)}), setting angle to 0.0°")
        else:
            angle = min(rect_angle, 180 - rect_angle)
            angle = angle + 55
        return cx, cy, w, h, angle

    def detect_objects(self, img):
        results = self.model(img, verbose=False)
        detected_objects = []
        rotboxes = []
        for result in results:
            boxes = result.boxes
            masks = result.masks
            if boxes is not None:
                for i in range(len(boxes.cls)):
                    score = float(boxes.conf[i])
                    if score < CONF_THRESHOLD:
                        continue
                    xyxy_coords = boxes.xyxy[i].flatten().tolist()
                    x_min, y_min, x_max, y_max = xyxy_coords[0], xyxy_coords[1], xyxy_coords[2], xyxy_coords[3]
                    cls_id = int(boxes.cls[i])
                    class_name = result.names[cls_id]
                    corners_8_coords = [x_min, y_min, x_max, y_min, x_max, y_max, x_min, y_max]
                    if corners_8_coords:
                        rotbox = RotBox(
                            cls=cls_id,
                            score=score,
                            x1=float(corners_8_coords[0]),
                            y1=float(corners_8_coords[1]),
                            x2=float(corners_8_coords[2]),
                            y2=float(corners_8_coords[3]),
                            x3=float(corners_8_coords[4]),
                            y3=float(corners_8_coords[5]),
                            x4=float(corners_8_coords[6]),
                            y4=float(corners_8_coords[7])
                        )
                        rotboxes.append(rotbox)
                        cx, cy, w, h, angle = self.process_rotbox(rotbox, img, class_name)
                        if cx is None:
                            continue
                    center_x_pixel = (x_min + x_max) / 2
                    center_y_pixel = (y_min + y_max) / 2
                    mm_coords = self.pixel_to_mm_transform([cx if cx is not None else center_x_pixel,
                                                            cy if cy is not None else center_y_pixel])
                    if mm_coords is None:
                        continue
                    mask_bbox = [x_min, y_min, x_max, y_max]
                    if masks is not None and masks.data is not None and len(masks.data) > i:
                        full_mask = masks.data[i].cpu().numpy().astype(np.uint8) * 255
                        object_mask = full_mask[int(y_min):int(y_max), int(x_min):int(x_max)]
                        if object_mask.size > 0 and cv2.countNonZero(object_mask) > 0:
                            y_coords, x_coords = np.where(object_mask > 0)
                            if len(x_coords) > 0 and len(y_coords) > 0:
                                mask_x1 = int(min(x_coords)) + int(x_min)
                                mask_y1 = int(min(y_coords)) + int(y_min)
                                mask_x2 = int(max(x_coords)) + int(x_min)
                                mask_y2 = int(max(y_coords)) + int(y_min)
                                mask_bbox = [mask_x1, mask_y1, mask_x2, mask_y2]
                    obj_data = {
                        "shape": class_name,
                        "pixel_center": [float(cx if cx is not None else center_x_pixel),
                                         float(cy if cy is not None else center_y_pixel)],
                        "mm_center": [float(mm_coords[0]), float(mm_coords[1])],
                        "angle_deg": float(angle),
                        "confidence": float(score),
                        "mask_bbox": mask_bbox,
                        "rotbox_size": [float(w if w is not None else 0.0),
                                        float(h if h is not None else 0.0)]
                    }
                    detected_objects.append(obj_data)
                    print(f"{class_name}: MM({mm_coords[0]:.1f},{mm_coords[1]:.1f})mm, "
                          f"Angle:{angle:.1f}°, BBox: {mask_bbox}, Size: [{w:.1f}, {h:.1f}]")
        return detected_objects

    def run(self):
        self.connect_camera()
        self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        converter = pylon.ImageFormatConverter()
        converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        self.running = True
        print("Place ArUco marker (100mm) and press 'c' to calibrate, 'q' to quit")
        while self.running:
            grab_result = self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            if grab_result.GrabSucceeded():
                image = converter.Convert(grab_result)
                img = image.GetArray().copy()
                key = cv2.waitKey(1) & 0xFF
                if not self.is_calibrated or key == ord('c'):
                    calibrated, marker_corners = self.calibrate_aruco_transform(img)
                corners, ids, _ = self.aruco_detector.detectMarkers(img)
                if corners and ids is not None and len(corners) > 0:
                    cv2.aruco.drawDetectedMarkers(img, corners, ids)
                    self.draw_aruco_axes(img, corners[0][0].reshape(4, 2))
                else:
                    self.draw_aruco_axes(img, None)
                if self.is_calibrated:
                    objects = self.detect_objects(img)
                    for obj in objects:
                        px, py = map(int, obj['pixel_center'])
                        mx, my = obj['mm_center']
                        cv2.circle(img, (px, py), 6, (0, 255, 0), -1)
                        cv2.putText(img, f"{obj['shape']} ({mx:.1f},{my:.1f})mm, "
                                        f"{obj['angle_deg']:.1f}°, BBox: {obj['mask_bbox']}, "
                                        f"Size: {obj['rotbox_size']}",
                                    (px, py - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                status = "CALIBRATED" if self.is_calibrated else "CALIBRATE"
                cv2.putText(img, f"Status: {status}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (0, 255, 0) if self.is_calibrated else (0, 0, 255), 2)
                cv2.imshow("ArUco Coordinate System", img)
                if self.is_calibrated and 'objects' in locals() and objects:
                    data_queue.put(objects)
                if key == ord('q'):
                    self.running = False
                elif key == ord('c'):
                    print("Recalibrating...")
            grab_result.Release()
        self.camera.StopGrabbing()
        self.camera.Close()
        cv2.destroyAllWindows()

    def stop(self):
        self.running = False
