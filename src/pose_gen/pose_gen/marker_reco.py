"""
"""
from typing import overload
from ultralytics import YOLO
import pyrealsense2

import cv2
import numpy as np
import os
import sys
import shutil
import subprocess
import importlib
from collections import deque
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Vector3Stamped, PolygonStamped, Point32, Point
from robo_arm_msgs.msg import PlaneFrame
from visualization_msgs.msg import Marker, MarkerArray

"""
相机到base_link的矩阵,后续要换成动态的TF
"""
CAMERA_TO_BASELINK_T = np.array([
    [0.0, 0.0, 1.0, 0.25],
    [-1.0, 0.0, 0.0, 0.0],
    [0.0, -1.0, 0.0, 0.4],
    [0.0, 0.0, 0.0, 1.0],
], dtype=np.float32)


"""
初始化颜色范围
"""
shared_color_ranges = {
    'red': [
        ((0, 100, 100), (10, 255, 255)),
        ((170, 100, 100), (180, 255, 255))
    ],
    'blue': [
        ((100, 100, 0), (140, 255, 255))
    ],
    'black': [
        ((0, 0, 0), (180, 255, 255))
    ],
    'white': [
        ((0, 0, 250), (180, 20, 255))
    ]
}


"""
模块、模型和视频流相关函数
"""
def parse_int_env(name, default):
    value = os.getenv(name)
    if value is None:
        return default
    try:
        return int(value)
    except ValueError:
        print(f"[WARN] Invalid {name}={value}, fallback to {default}")
        return default


def patch_numpy_bitgenerator_unpickle_compat():
    try:
        np_pickle = importlib.import_module('numpy.random._pickle')
    except Exception:
        return

    if getattr(np_pickle, '_hkust_patched_ctor', False):
        return

    class _DummyRngState:
        def __init__(self, *args, **kwargs):
            self.state = None

        def __setstate__(self, state):
            self.state = state

    np_pickle.__bit_generator_ctor = lambda *args, **kwargs: _DummyRngState()
    np_pickle.__generator_ctor = lambda *args, **kwargs: _DummyRngState()
    np_pickle.__randomstate_ctor = lambda *args, **kwargs: _DummyRngState()
    np_pickle._hkust_patched_ctor = True


def _resolve_yolo_class_name(names, class_id):
    if names is None:
        return None
    try:
        if isinstance(names, dict):
            return names.get(int(class_id))
        if isinstance(names, (list, tuple)):
            return names[int(class_id)]
    except Exception:
        return None
    return None


def predict_marker_roi(model, image_mat, conf_thr=0.6, iou_thr=0.45, imgsz=640, roi_expand_ratio=0.10):
    if model is None or image_mat is None:
        return None
    try:
        results = model.predict(
            source=image_mat,
            conf=float(conf_thr),
            iou=float(iou_thr),
            imgsz=int(imgsz),
            verbose=False,
        )
    except Exception as exc:
        print(f'[WARN] OBB inference failed: {exc}')
        return None

    if not results:
        return None
   
    result = results[0]

    names_map = getattr(result, 'names', None)
    if names_map is None:
        names_map = getattr(model, 'names', None)

    detected_classes = []

    """
    boxes = getattr(result, 'boxes', None)
    if boxes is not None and boxes.cls is not None:
        try:
            class_ids = boxes.cls.cpu().numpy().astype(np.int32).tolist()
        except Exception:
            class_ids = np.asarray(boxes.cls).astype(np.int32).tolist()
        for cid in class_ids:
            cname = _resolve_yolo_class_name(names_map, cid)
            detected_classes.append(cname if cname is not None else f'class_{int(cid)}')
    else:
    """
    obb = getattr(result, 'obb', None)
    if obb is not None and obb.cls is not None:
        try:
            class_ids = obb.cls.cpu().numpy().astype(np.int32).tolist()
        except Exception:
            class_ids = np.asarray(obb.cls).astype(np.int32).tolist()
        for cid in class_ids:
            cname = _resolve_yolo_class_name(names_map, cid)
            detected_classes.append(cname if cname is not None else f'class_{int(cid)}')

    names = names_map

    """
    if boxes is not None and boxes.xyxy is not None and boxes.cls is not None:
        try:
            xyxy = boxes.xyxy.cpu().numpy()
            confs = boxes.conf.cpu().numpy() if boxes.conf is not None else np.ones((len(xyxy),), dtype=np.float32)
            classes = boxes.cls.cpu().numpy()
        except Exception:
            xyxy = np.asarray(boxes.xyxy)
            confs = np.asarray(boxes.conf) if boxes.conf is not None else np.ones((len(xyxy),), dtype=np.float32)
            classes = np.asarray(boxes.cls)

        pillar_indices = []
        for idx, class_id in enumerate(classes):
            class_name = _resolve_yolo_class_name(names, class_id)
            if class_name == 'pillar':
                pillar_indices.append(idx)

        if pillar_indices:
            best_idx = max(pillar_indices, key=lambda idx: float(confs[idx]))
            best_conf = float(confs[best_idx])
            if best_conf < float(conf_thr):
                return None
            x1, y1, x2, y2 = [float(v) for v in xyxy[best_idx]]
            clipped = _clip_box_xyxy((x1, y1, x2, y2), image_mat.shape)
            if clipped is None:
                return None
            return _expand_box_xyxy(clipped, image_mat.shape, roi_expand_ratio)
    """
    obb = getattr(result, 'obb', None)
    if obb is None or obb.xyxyxyxy is None:
        return None

    try:
        polygons = obb.xyxyxyxy.cpu().numpy()
        confs = obb.conf.cpu().numpy() if obb.conf is not None else np.ones((len(polygons),), dtype=np.float32)
        classes = obb.cls.cpu().numpy() if obb.cls is not None else np.zeros((len(polygons),), dtype=np.float32)
    except Exception:
        polygons = np.asarray(obb.xyxyxyxy)
        confs = np.asarray(obb.conf) if obb.conf is not None else np.ones((len(polygons),), dtype=np.float32)
        classes = np.asarray(obb.cls) if obb.cls is not None else np.zeros((len(polygons),), dtype=np.float32)

    if len(polygons) == 0:
        return None

    pillar_indices = []
    for idx, class_id in enumerate(classes):
        class_name = _resolve_yolo_class_name(names, class_id)
        if class_name == 'pillar':
            pillar_indices.append(idx)

    if not pillar_indices:
        return None

    best_idx = max(pillar_indices, key=lambda idx: float(confs[idx]))
    best_conf = float(confs[best_idx])
    if best_conf < float(conf_thr):
        return None

    poly = np.asarray(polygons[best_idx], dtype=np.float32).reshape(-1, 2)
    if poly.shape[0] != 4:
        return None

    x1 = float(np.min(poly[:, 0]))
    y1 = float(np.min(poly[:, 1]))
    x2 = float(np.max(poly[:, 0]))
    y2 = float(np.max(poly[:, 1]))
    clipped = _clip_box_xyxy((x1, y1, x2, y2), image_mat.shape)
    if clipped is None:
        return None
    return _expand_box_xyxy(clipped, image_mat.shape, roi_expand_ratio)


def resolve_rs_enumerate_devices_path():
    # Priority: env override -> /usr/local -> ROS Jazzy -> PATH
    env_path = os.getenv("RS_ENUMERATE_BIN")
    candidates = [
        env_path,
        "/usr/local/bin/rs-enumerate-devices",
        "/opt/ros/jazzy/bin/rs-enumerate-devices",
        "rs-enumerate-devices",
    ]

    for candidate in candidates:
        if not candidate:
            continue
        if os.path.isabs(candidate):
            if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
                return candidate
            continue
        resolved = shutil.which(candidate)
        if resolved:
            return resolved

    return None


def probe_realsense_devices(timeout_sec=5):
    cmd = resolve_rs_enumerate_devices_path()
    if cmd is None:
        print("[WARN] rs-enumerate-devices not found; skip external probe.")
        return False

    print(f"[INFO] Probe RealSense with: {cmd}")
    try:
        result = subprocess.run(
            [cmd],
            capture_output=True,
            text=True,
            timeout=timeout_sec,
            check=False,
        )
    except Exception as exc:
        print(f"[WARN] Failed to run rs-enumerate-devices: {exc}")
        return False

    output = (result.stdout or "") + (result.stderr or "")
    output_lower = output.lower()
    if result.returncode != 0:
        print(f"[WARN] rs-enumerate-devices exited with {result.returncode}")
        return False

    # Treat probe as successful only when output indicates an actual connected device.
    if "device count" in output_lower and "device count: 0" in output_lower:
        print("[WARN] RealSense probe found 0 devices.")
        return False

    if "device info" in output_lower or "serial number" in output_lower:
        print("[INFO] RealSense probe found at least one device.")
        return True

    print("[WARN] Probe command ran, but no connected RealSense device was identified.")
    return False


def open_realsense_pipeline():
    rs_module = pyrealsense2
    if rs_module is None:
        print("[WARN] pyrealsense2 is not available.")
        return None

    if not probe_realsense_devices():
        print("[WARN] RealSense probe failed. Check cable, USB3 port, and permissions.")
        return None

    width = parse_int_env("REALSENSE_WIDTH", 640)
    height = parse_int_env("REALSENSE_HEIGHT", 480)
    fps = parse_int_env("REALSENSE_FPS", 30)
    serial = os.getenv("REALSENSE_SERIAL")

    pipeline = rs_module.pipeline()
    config = rs_module.config()

    if serial:
        config.enable_device(serial)

    config.enable_stream(rs_module.stream.color, width, height, rs_module.format.bgr8, fps)
    config.enable_stream(rs_module.stream.depth, width, height, rs_module.format.z16, fps)

    try:
        pipeline.start(config)
    except Exception as exc:
        print(f"[WARN] Failed to start RealSense pipeline: {exc}")
        return None

    print(f"[INFO] RealSense opened: {width}x{height}@{fps}")
    if serial:
        print(f"[INFO] RealSense serial: {serial}")
    return pipeline


def read_realsense_color_frame(pipeline, align=None, timeout_ms=2000):
    try:
        frames = pipeline.wait_for_frames(timeout_ms)
    except Exception as exc:
        print(f"[WARN] wait_for_frames failed: {exc}")
        return False, None, None, None

    if align is not None:
        frames = align.process(frames)

    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()
    if not color_frame or not depth_frame:
        return False, None, None, None

    image_mat = np.asanyarray(color_frame.get_data())
    return True, image_mat, color_frame, depth_frame


"""
算法函数：几何、图像处理和坐标计算
"""
def _polygon_center_2d(polygon):
    pts = polygon.reshape(-1, 2).astype(np.float32)
    poly_i32 = pts.astype(np.int32)
    m = cv2.moments(poly_i32)
    if m["m00"] != 0:
        return np.asarray([m["m10"] / m["m00"], m["m01"] / m["m00"]], dtype=np.float32)
    x1 = float(np.min(pts[:, 0]))
    y1 = float(np.min(pts[:, 1]))
    x2 = float(np.max(pts[:, 0]))
    y2 = float(np.max(pts[:, 1]))
    return np.asarray([(x1 + x2) * 0.5, (y1 + y2) * 0.5], dtype=np.float32)


def _order_points_clockwise_from_topleft(points_xy):
    if points_xy is None:
        return None, None

    pts = np.asarray(points_xy, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] != 2 or len(pts) == 0:
        return None, None

    # Use box center, not arithmetic mean, to avoid centroid bias.
    c_x = 0.5 * (float(np.min(pts[:, 0])) + float(np.max(pts[:, 0])))
    c_y = 0.5 * (float(np.min(pts[:, 1])) + float(np.max(pts[:, 1])))
    angles = np.arctan2(pts[:, 1] - c_y, pts[:, 0] - c_x)
    order = np.argsort(angles)
    ordered = pts[order]

    # Deterministic start: top-left point.
    start_idx = int(np.argmin(np.sum(ordered, axis=1)))
    ordered = np.roll(ordered, -start_idx, axis=0) #把start_idx行的元素移到最前面，其他行依次向后移动
    order = np.roll(order, -start_idx, axis=0)
    return ordered, order

def _extract_vertex_from_hex_2d(hexagon, box_center_xy):
    pts = hexagon.reshape(-1, 2).astype(np.float32)
    c = _polygon_center_2d(hexagon)
    direction = c - np.asarray(box_center_xy, dtype=np.float32)
    d_norm = float(np.linalg.norm(direction))
    if d_norm < 1e-6:
        idx = int(np.argmax(np.sum((pts - c) ** 2, axis=1)))
        return pts[idx]
    direction = direction / d_norm
    scores = (pts - c) @ direction
    idx = int(np.argmax(scores))
    return pts[idx]


def _fit_plane_svd(points_3d):
    if points_3d is None or len(points_3d) < 3:
        return None, None
    centroid = np.mean(points_3d, axis=0)
    centered = points_3d - centroid
    _, s, vh = np.linalg.svd(centered, full_matrices=False)
    if len(s) < 3:
        return None, None
    q_axis = vh[-1, :]
    n = np.linalg.norm(q_axis)
    if n < 1e-8:
        return None, None
    return centroid, q_axis / n


def _clip_box_xyxy(box_xyxy, image_shape):
    height, width = image_shape[:2]
    x1, y1, x2, y2 = box_xyxy
    x1 = max(0, min(int(round(x1)), width - 1))
    y1 = max(0, min(int(round(y1)), height - 1))
    x2 = max(0, min(int(round(x2)), width - 1))
    y2 = max(0, min(int(round(y2)), height - 1))
    if x2 <= x1 or y2 <= y1:
        return None
    return x1, y1, x2, y2


def _expand_box_xyxy(box_xyxy, image_shape, expand_ratio):
    x1, y1, x2, y2 = box_xyxy
    w = max(1.0, float(x2 - x1))
    h = max(1.0, float(y2 - y1))
    cx = 0.5 * (x1 + x2)
    cy = 0.5 * (y1 + y2)
    half_w = 0.5 * w * (1.0 + max(0.0, float(expand_ratio)))
    half_h = 0.5 * h * (1.0 + max(0.0, float(expand_ratio)))
    expanded = (cx - half_w, cy - half_h, cx + half_w, cy + half_h)
    return _clip_box_xyxy(expanded, image_shape)



def sharpen_image(img, sigma=1.0):
    blurred = cv2.GaussianBlur(img, (0, 0), sigma)
    sharpened = cv2.addWeighted(img, 1.5, blurred, -0.5, 0)
    return sharpened


def hsv_mask(img, lower_hsv, upper_hsv):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    masked_img = cv2.bitwise_and(img, img, mask=mask)
    return masked_img


def find_hexagon_contours(binary_img, min_area=500, epsilon_ratio=0.02):
    contours, _ = cv2.findContours(binary_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    hexagons = []

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue
        perimeter = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon_ratio * perimeter, True) #基于Ramer-Douglas-Peucker算法
        if len(approx) == 6:
            hexagons.append((area, approx))

    hexagons.sort(key=lambda x: x[0], reverse=True)
    return hexagons

def order_hexagons_by_semantic_position(hexagons, roi_box_xyxy=None):
    hex_semantic = [None] * 4
    if hexagons is None or len(hexagons) == 0:
        return hex_semantic

    valid_items = []
    valid_centers = []
    for item in hexagons:
        polygon = item[1]
        if polygon is None:
            continue
        valid_items.append(item)
        valid_centers.append(_polygon_center_2d(polygon))

    if len(valid_items) == 0:
        return hex_semantic

    centers = np.asarray(valid_centers, dtype=np.float32)
    box_center = np.asarray([
        0.5 * (float(np.min(centers[:, 0])) + float(np.max(centers[:, 0]))),
        0.5 * (float(np.min(centers[:, 1])) + float(np.max(centers[:, 1]))),
    ], dtype=np.float32)
    for i, item in enumerate(valid_items):
        _, hexagon = item[:2]
        center = _polygon_center_2d(hexagon)
        left = center[0] <= box_center[0]
        up = center[1] <= box_center[1]
        if up and left:
            hex_semantic[0] = (item[0], hexagon, 'UL')
        elif up and (not left):
            hex_semantic[1] = (item[0], hexagon, 'UR')
        elif (not up) and left:
            hex_semantic[2] = (item[0], hexagon, 'LL')
        else:
            hex_semantic[3] = (item[0], hexagon, 'LR')

    return hex_semantic


def order_hexagons_clockwise_from_topleft(hexagons):
    if hexagons is None or len(hexagons) == 0:
        return []

    normalized = []
    centers = []
    for item in hexagons:
        polygon = item[1]
        if polygon is None:
            continue
        normalized.append(item)
        centers.append(_polygon_center_2d(polygon))

    if len(normalized) == 0:
        return []

    _, order = _order_points_clockwise_from_topleft(centers)
    if order is None:
        return list(normalized)
    return [normalized[int(idx)] for idx in order]


def deproject_pixel_to_3d(rs_module, color_frame, depth_frame, u, v):
    if rs_module is None or color_frame is None or depth_frame is None:
        return None
    try:
        intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
    except Exception:
        return None

    width = color_frame.get_width()
    height = color_frame.get_height()
    uu = max(0, min(int(round(float(u))), width - 1))
    vv = max(0, min(int(round(float(v))), height - 1))
    depth = depth_frame.get_distance(uu, vv)
    if depth <= 0.0:
        return None
    point = rs_module.rs2_deproject_pixel_to_point(intrinsics, [float(uu), float(vv)], float(depth))
    return np.asarray(point, dtype=np.float32)


def project_point_to_pixel(rs_module, color_frame, point_3d):
    if rs_module is None or color_frame is None or point_3d is None:
        return None
    try:
        intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
        uv = rs_module.rs2_project_point_to_pixel(
            intrinsics,
            [float(point_3d[0]), float(point_3d[1]), float(point_3d[2])],
        )
    except Exception:
        return None
    return np.asarray(uv, dtype=np.float32)


def filter_hexagons_in_roi(hexagons, roi_box_xyxy):
    if roi_box_xyxy is None: return hexagons
    x1, y1, x2, y2 = roi_box_xyxy
    filtered = []
    for area, hexagon in hexagons:
        pts = hexagon.reshape(-1, 2).astype(np.float32)
        center = np.mean(pts, axis=0)
        cx, cy = float(center[0]), float(center[1])
        if x1 <= cx <= x2 and y1 <= cy <= y2:
            filtered.append((area, hexagon))
    return filtered


def filter_hexagons_in_roi_strict(hexagons, roi_box_xyxy):
    if roi_box_xyxy is None: return hexagons
    x1, y1, x2, y2 = roi_box_xyxy
    filtered = []
    for area, hexagon in hexagons:
        pts = hexagon.reshape(-1, 2).astype(np.float32)
        for pt in pts:
            if not (x1 <= pt[0] <= x2 and y1 <= pt[1] <= y2):
                break
        else:
            filtered.append((area, hexagon))
    return filtered


def compute_p_up_axes(points_3d):
    if points_3d is None or len(points_3d) != 4:
        return None, None

    p0, p1, p2, p3 = points_3d
    horizontal_1 = p1 - p0
    horizontal_2 = p2 - p3
    vertical_1 = p3 - p0
    vertical_2 = p2 - p1

    p_axis = horizontal_1 + horizontal_2
    up_axis = vertical_1 + vertical_2

    p_norm = np.linalg.norm(p_axis) #linalg is short for linear algebra
    up_norm = np.linalg.norm(up_axis)
    if p_norm < 1e-8 or up_norm < 1e-8:
        return None, None

    #p_axis指向marker右边，up_axis指向marker上边
    return p_axis / p_norm, up_axis / up_norm


def estimate_plane_and_axes_from_hexagons(hexagons, rs_module, color_frame, depth_frame):
    if hexagons is None:
        return None, None, None, None

    selected = []
    centers_2d = []
    for item in hexagons:
        if item is None:
            continue
        polygon = item[1]
        if polygon is None:
            continue
        label = item[2] if len(item) >= 3 else None
        selected.append((label, polygon))
        centers_2d.append(_polygon_center_2d(polygon))

    if len(selected) < 3:
        return None, None, None, None

    box_center_2d = np.asarray([
        0.5 * (float(np.min([c[0] for c in centers_2d])) + float(np.max([c[0] for c in centers_2d]))),
        0.5 * (float(np.min([c[1] for c in centers_2d])) + float(np.max([c[1] for c in centers_2d]))),
    ], dtype=np.float32)

    vertices_3d = []
    points_by_label = {}
    for label, hexagon in selected:
        vertex_2d = _extract_vertex_from_hex_2d(hexagon, box_center_2d)
        point_3d = deproject_pixel_to_3d(rs_module, color_frame, depth_frame, vertex_2d[0], vertex_2d[1])
        if point_3d is not None:
            vertices_3d.append(point_3d)
            if label is not None:
                points_by_label[label] = point_3d

    if len(vertices_3d) < 3:
        return None, None, None, None

    centers = []
    if 'UL' in points_by_label and 'LR' in points_by_label:
        centers.append(0.5 * (points_by_label['UL'] + points_by_label['LR']))
    if 'UR' in points_by_label and 'LL' in points_by_label:
        centers.append(0.5 * (points_by_label['UR'] + points_by_label['LL']))
    if centers:
        center_3d = np.mean(np.stack(centers, axis=0), axis=0).astype(np.float32)
    else:
        points = list(points_by_label.values())
        if len(points) == 0:
            return None, None, None, None
        center_3d = np.mean(np.stack(points, axis=0), axis=0).astype(np.float32)

    horizontal_vectors = []
    if 'UL' in points_by_label and 'UR' in points_by_label:
        horizontal_vectors.append(points_by_label['UR'] - points_by_label['UL'])
    if 'LL' in points_by_label and 'LR' in points_by_label:
        horizontal_vectors.append(points_by_label['LR'] - points_by_label['LL'])

    vertical_vectors = []
    if 'UL' in points_by_label and 'LL' in points_by_label:
        vertical_vectors.append(points_by_label['UL'] - points_by_label['LL'])
    if 'UR' in points_by_label and 'LR' in points_by_label:
        vertical_vectors.append(points_by_label['UR'] - points_by_label['LR'])

    p_vectors = []
    for vector in horizontal_vectors:
        vector = np.asarray(vector, dtype=np.float32)
        norm = float(np.linalg.norm(vector))
        if norm > 1e-8:
            p_vectors.append(vector / norm)

    up_vectors = []
    for vector in vertical_vectors:
        vector = np.asarray(vector, dtype=np.float32)
        norm = float(np.linalg.norm(vector))
        if norm > 1e-8:
            up_vectors.append(vector / norm)

    if not p_vectors or not up_vectors:
        return None, None, None, None

    p_axis = np.sum(np.stack(p_vectors, axis=0), axis=0)
    up_axis = np.sum(np.stack(up_vectors, axis=0), axis=0)

    p_norm = float(np.linalg.norm(p_axis))
    up_norm = float(np.linalg.norm(up_axis))
    if p_norm < 1e-8 or up_norm < 1e-8:
        return None, None, None, None

    p_axis = (p_axis / p_norm).astype(np.float32)
    up_axis = (up_axis / up_norm).astype(np.float32)
    q_axis = np.cross(p_axis, up_axis)
    q_norm = float(np.linalg.norm(q_axis))
    if q_norm < 1e-8:
        return None, None, None, None
    q_axis = (q_axis / q_norm).astype(np.float32)

    return vertices_3d, center_3d.astype(np.float32), q_axis, (p_axis.astype(np.float32), up_axis.astype(np.float32))


def transform_points_and_q_to_baselink(points_3d, center_3d, q_axis):
    if points_3d is None or len(points_3d) < 3 or center_3d is None or q_axis is None:
        return None, None, None

    transform = CAMERA_TO_BASELINK_T
    rotation = transform[:3, :3]
    translation = transform[:3, 3]

    transformed_points = np.asarray([
        (rotation @ point) + translation for point in points_3d
    ], dtype=np.float32)

    transformed_center = (rotation @ center_3d) + translation
    transformed_center = np.asarray(transformed_center, dtype=np.float32)

    transformed_q = rotation @ q_axis
    norm = np.linalg.norm(transformed_q)
    if norm < 1e-8:
        return None, None, None
    transformed_q = transformed_q / norm

    return transformed_points, transformed_center, transformed_q


"""
绘制函数
"""
def draw_hexagons(img, hexagons):
    out = img.copy()

    for i, (_, hexagon) in enumerate(hexagons, start=1):
        poly = hexagon.astype(np.int32)
        cv2.polylines(out, [poly], True, (255, 255, 0), 2)
        center = _polygon_center_2d(hexagon)
        cx, cy = int(center[0]), int(center[1])

    return out


def draw_roi_box(image, roi_box_xyxy):
    if roi_box_xyxy is None:
        return image
    out = image.copy()
    x1, y1, x2, y2 = roi_box_xyxy
    cv2.rectangle(out, (x1, y1), (x2, y2), (0, 128, 255), 2)
    cv2.putText(out, 'NN ROI', (x1 + 4, max(20, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 128, 255), 2)
    return out


def draw_hexagon_plane_overlay(img, selected_hexagons, plane_center_px=None):
    out = img.copy()
    if selected_hexagons is None or len(selected_hexagons) == 0:
        return out

    semantic_hex = [
        selected_hexagons[idx]
        for idx in range(min(4, len(selected_hexagons)))
        if selected_hexagons[idx] is not None and selected_hexagons[idx][1] is not None
    ]
    if len(semantic_hex) == 0:
        return out

    centers = []
    for idx, item in enumerate(semantic_hex):
        hexagon = item[1]
        semantic_label = item[2] if len(item) >= 3 else None
        center = _polygon_center_2d(hexagon)
        centers.append(center)
        cx, cy = int(center[0]), int(center[1])
        cv2.circle(out, (cx, cy), 4, (0, 200, 255), -1)
        quadratic_label = semantic_label if semantic_label is not None else '?'
        cv2.putText(out, quadratic_label, (cx + 5, cy + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 120), 1)

    centers = np.asarray(centers, dtype=np.float32)
    plane_center = None
    if plane_center_px is not None:
        plane_center = np.asarray(plane_center_px, dtype=np.float32)
        label = 'PlaneCenter(3D)'

    if plane_center is not None:
        pcx, pcy = int(plane_center[0]), int(plane_center[1])
        cv2.circle(out, (pcx, pcy), 6, (0, 0, 255), -1)
        cv2.putText(out, label, (pcx + 6, pcy - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    if len(centers) >= 4 and plane_center is not None:
        rel = centers - plane_center
        angles = np.arctan2(rel[:, 1], rel[:, 0])
        order = np.argsort(angles)
        ordered = centers[order][:4]

        poly = ordered.astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(out, [poly], True, (0, 165, 255), 2)
        cv2.line(out, tuple(ordered[0].astype(np.int32)), tuple(ordered[2].astype(np.int32)), (255, 0, 255), 2)
        cv2.line(out, tuple(ordered[1].astype(np.int32)), tuple(ordered[3].astype(np.int32)), (0, 255, 255), 2)

    return out


"""
ROS 节点
"""
class MarkerRecoNode(Node):
    def __init__(self):
        super().__init__('marker_reco_node')

        self.frame_id = 'base_link'
        self.filtered_up_topic = '/marker_reco/filtered/up_direction'
        self.filtered_p_topic = '/marker_reco/filtered/p_direction'
        self.filtered_q_topic = '/marker_reco/filtered/q_direction'
        self.filtered_corners_topic = '/marker_reco/filtered/corners_3d'
        self.filtered_center_topic = '/marker_reco/filtered/center_3d'
        self.filtered_plane_topic = '/marker_reco/filtered/plane'
        self.viz_topic = '/marker_reco/visualization'
        self.q_arrow_length = 0.12
        self.axis_arrow_length = 0.10
        self.show_debug_windows = True
        self.timer_period_sec = 0.1
        self.obb_model_path = '/home/hku/robo_arm/src/pose_gen/weights/hkust_best.pt'
        self.obb_conf_thr = 0.6
        self.obb_iou_thr = 0.6
        self.obb_imgsz = 640
        self.roi_expand_ratio = 0.02
        self.debug_log_every_n_frames = 15

        self.filtered_up_pub = self.create_publisher(Vector3Stamped, self.filtered_up_topic, 10)
        self.filtered_p_pub = self.create_publisher(Vector3Stamped, self.filtered_p_topic, 10)
        self.filtered_q_pub = self.create_publisher(Vector3Stamped, self.filtered_q_topic, 10)
        self.filtered_corners_pub = self.create_publisher(PolygonStamped, self.filtered_corners_topic, 10)
        self.filtered_center_pub = self.create_publisher(PointStamped, self.filtered_center_topic, 10)
        self.filtered_plane_pub = self.create_publisher(PlaneFrame, self.filtered_plane_topic, 10)
        self.viz_pub = self.create_publisher(MarkerArray, self.viz_topic, 10)

        self.filter_window_size = max(5, parse_int_env('MARKER_FILTER_WINDOW', 20))
        self.filter_cosine_threshold = float(os.getenv('MARKER_FILTER_COSINE_THR', '0.94'))
        self.filter_point_distance_threshold = float(os.getenv('MARKER_FILTER_POINT_THR', '0.03'))
        self.filtered_window = deque(maxlen=self.filter_window_size)

        self.rs_module = pyrealsense2
        self.pipeline = open_realsense_pipeline()
        self.align = None

        if self.pipeline is None:
            raise RuntimeError('RealSense L515 is required, but no stream could be started.')

        if self.rs_module is not None:
            self.align = self.rs_module.align(self.rs_module.stream.color)

        patch_numpy_bitgenerator_unpickle_compat()
        try:
            self.obb_model = YOLO(self.obb_model_path)
        except Exception as exc:
            self.get_logger().warn(f'Failed to load OBB model {self.obb_model_path}: {exc}')
        if self.obb_model is None:
            self.get_logger().warn('NN ROI is enabled but model is unavailable; fallback to contour-only mode')

        self.kernel = np.ones((3, 3), np.uint8)
        self._debug_frame_counter = 0
        self.timer = self.create_timer(self.timer_period_sec, self.timer_callback)
        self.get_logger().info(
            f'marker_reco_node started, publish frame {self.frame_id}, '
            f'filtered to /marker_reco/filtered/*, viz to {self.viz_topic}'
        )

    @staticmethod
    def _normalize_vector(v):
        norm = float(np.linalg.norm(v))
        if norm < 1e-8:
            return None
        return (v / norm).astype(np.float32)

    @staticmethod
    def _filtered_average_direction(samples, cosine_threshold):
        if not samples:
            return None

        aligned = []
        reference = None
        for s in samples:
            n = MarkerRecoNode._normalize_vector(np.asarray(s, dtype=np.float32))
            if n is not None:
                reference = n
                break
        if reference is None:
            return None

        for s in samples:
            n = MarkerRecoNode._normalize_vector(np.asarray(s, dtype=np.float32))
            if n is None:
                continue
            if float(np.dot(n, reference)) < 0.0:
                n = -n
            aligned.append(n)

        if not aligned:
            return None

        mean = np.mean(np.stack(aligned, axis=0), axis=0)
        mean_n = MarkerRecoNode._normalize_vector(mean)
        if mean_n is None:
            return None

        inliers = [v for v in aligned if float(np.dot(v, mean_n)) >= cosine_threshold]
        selected = inliers if inliers else aligned
        out = np.mean(np.stack(selected, axis=0), axis=0)
        return MarkerRecoNode._normalize_vector(out)

    @staticmethod
    def _filtered_average_point(samples, distance_threshold):
        if not samples:
            return None
        arr = np.asarray(samples, dtype=np.float32)
        arr = arr[np.isfinite(arr).all(axis=1)]
        if arr.shape[0] == 0:
            return None

        mean = np.mean(arr, axis=0)
        dist = np.linalg.norm(arr - mean, axis=1)
        inlier_mask = dist <= distance_threshold
        inliers = arr[inlier_mask]
        if inliers.shape[0] == 0:
            inliers = arr

        out = np.mean(inliers, axis=0)
        return out.astype(np.float32)

    def _publish_filtered_geometry(self, stamp, center, q_axis, p_axis, up_axis, corners):
        plane_msg = PlaneFrame()
        plane_msg.header.stamp = stamp
        plane_msg.header.frame_id = self.frame_id
        plane_msg.p_axis.x = float(p_axis[0])
        plane_msg.p_axis.y = float(p_axis[1])
        plane_msg.p_axis.z = float(p_axis[2])
        plane_msg.q_axis.x = float(q_axis[0])
        plane_msg.q_axis.y = float(q_axis[1])
        plane_msg.q_axis.z = float(q_axis[2])
        plane_msg.up_axis.x = float(up_axis[0])
        plane_msg.up_axis.y = float(up_axis[1])
        plane_msg.up_axis.z = float(up_axis[2])
        plane_msg.center.x = float(center[0])
        plane_msg.center.y = float(center[1])
        plane_msg.center.z = float(center[2])
        for point in corners:
            corner = Point32()
            corner.x = float(point[0])
            corner.y = float(point[1])
            corner.z = float(point[2])
            plane_msg.corners.append(corner)
        self.filtered_plane_pub.publish(plane_msg)

        up_msg = Vector3Stamped()
        up_msg.header.stamp = stamp
        up_msg.header.frame_id = self.frame_id
        up_msg.vector.x = float(up_axis[0])
        up_msg.vector.y = float(up_axis[1])
        up_msg.vector.z = float(up_axis[2])
        self.filtered_up_pub.publish(up_msg)

        p_msg = Vector3Stamped()
        p_msg.header.stamp = stamp
        p_msg.header.frame_id = self.frame_id
        p_msg.vector.x = float(p_axis[0])
        p_msg.vector.y = float(p_axis[1])
        p_msg.vector.z = float(p_axis[2])
        self.filtered_p_pub.publish(p_msg)

        q_msg = Vector3Stamped()
        q_msg.header.stamp = stamp
        q_msg.header.frame_id = self.frame_id
        q_msg.vector.x = float(q_axis[0])
        q_msg.vector.y = float(q_axis[1])
        q_msg.vector.z = float(q_axis[2])
        self.filtered_q_pub.publish(q_msg)

        center_msg = PointStamped()
        center_msg.header.stamp = stamp
        center_msg.header.frame_id = self.frame_id
        center_msg.point.x = float(center[0])
        center_msg.point.y = float(center[1])
        center_msg.point.z = float(center[2])
        self.filtered_center_pub.publish(center_msg)

        corners_msg = PolygonStamped()
        corners_msg.header.stamp = stamp
        corners_msg.header.frame_id = self.frame_id
        for point in corners:
            corner = Point32()
            corner.x = float(point[0])
            corner.y = float(point[1])
            corner.z = float(point[2])
            corners_msg.polygon.points.append(corner)
        self.filtered_corners_pub.publish(corners_msg)

    def _update_and_publish_filtered(self, stamp, center_3d, q_axis, p_axis, up_axis, points_3d):
        if points_3d is None or len(points_3d) < 3:
            return

        frame = {
            'q': np.asarray(q_axis, dtype=np.float32),
            'p': np.asarray(p_axis, dtype=np.float32),
            'up': np.asarray(up_axis, dtype=np.float32),
            'center': np.asarray(center_3d, dtype=np.float32),
            'corners': np.asarray(points_3d, dtype=np.float32),
        }
        self.filtered_window.append(frame)

        q_avg = self._filtered_average_direction(
            [f['q'] for f in self.filtered_window],
            self.filter_cosine_threshold,
        )
        p_avg = self._filtered_average_direction(
            [f['p'] for f in self.filtered_window],
            self.filter_cosine_threshold,
        )
        up_avg = self._filtered_average_direction(
            [f['up'] for f in self.filtered_window],
            self.filter_cosine_threshold,
        )
        center_avg = self._filtered_average_point(
            [f['center'] for f in self.filtered_window],
            self.filter_point_distance_threshold,
        )

        if q_avg is None or p_avg is None or up_avg is None or center_avg is None:
            return

        p_avg = p_avg - np.dot(p_avg, q_avg) * q_avg
        p_avg = self._normalize_vector(p_avg)
        if p_avg is None:
            return

        up_avg = up_avg - np.dot(up_avg, q_avg) * q_avg
        up_avg = up_avg - np.dot(up_avg, p_avg) * p_avg
        up_avg = self._normalize_vector(up_avg)
        if up_avg is None:
            up_avg = self._normalize_vector(np.cross(q_avg, p_avg))
            if up_avg is None:
                return

        orthogonal_up = self._normalize_vector(np.cross(q_avg, p_avg))
        if orthogonal_up is None:
            return
        if float(np.dot(orthogonal_up, up_avg)) < 0.0:
            orthogonal_up = -orthogonal_up
        up_avg = orthogonal_up

        corner_count = int(frame['corners'].shape[0])
        corner_avg = []
        for i in range(corner_count):
            corner_samples = [
                f['corners'][i]
                for f in self.filtered_window
                if f['corners'].shape[0] > i
            ]
            one_corner = self._filtered_average_point(corner_samples, self.filter_point_distance_threshold)
            if one_corner is None:
                return
            corner_avg.append(one_corner)

        corner_avg = np.asarray(corner_avg, dtype=np.float32)
        self._publish_filtered_geometry(stamp, center_avg, q_avg, p_avg, up_avg, corner_avg)

    def publish_plane_geometry(self, points_3d, center_3d, q_axis, p_axis=None, up_axis=None):
        points_3d, center_3d, q_axis = transform_points_and_q_to_baselink(points_3d, center_3d, q_axis)
        if points_3d is None or center_3d is None or q_axis is None:
            self.get_logger().warn('Failed to transform plane geometry to base_link')
            return

        if p_axis is not None and up_axis is not None:
            rotation = CAMERA_TO_BASELINK_T[:3, :3]
            p_axis = rotation @ p_axis
            up_axis = rotation @ up_axis
            p_norm = float(np.linalg.norm(p_axis))
            up_norm = float(np.linalg.norm(up_axis))
            if p_norm > 1e-8 and up_norm > 1e-8:
                p_axis = p_axis / p_norm
                up_axis = up_axis / up_norm
            else:
                p_axis, up_axis = None, None

        if p_axis is None or up_axis is None:
            p_axis, up_axis = compute_p_up_axes(points_3d)

        if p_axis is None or up_axis is None:
            self.get_logger().warn('Failed to compute P/V axes for plane geometry')
            return

        stamp = self.get_clock().now().to_msg()

        self._update_and_publish_filtered(stamp, center_3d, q_axis, p_axis, up_axis, points_3d)

        self.publish_visualization_markers(points_3d, center_3d, q_axis, p_axis, up_axis, stamp)

    def _build_arrow_marker(self, marker_id, stamp, center, end, color_rgb):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = 'marker_reco'
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.01
        marker.scale.y = 0.02
        marker.scale.z = 0.03
        marker.color.r = float(color_rgb[0])
        marker.color.g = float(color_rgb[1])
        marker.color.b = float(color_rgb[2])
        marker.color.a = 1.0

        start_pt = Point()
        start_pt.x = float(center[0])
        start_pt.y = float(center[1])
        start_pt.z = float(center[2])

        end_pt = Point()
        end_pt.x = float(end[0])
        end_pt.y = float(end[1])
        end_pt.z = float(end[2])

        marker.points = [start_pt, end_pt]
        return marker

    def publish_visualization_markers(self, points_3d, center, q_axis, p_axis, up_axis, stamp):
        q_end = center + (q_axis * float(self.q_arrow_length))
        p_end = center + (p_axis * float(self.axis_arrow_length))
        up_end = center + (up_axis * float(self.axis_arrow_length))

        marker_array = MarkerArray()

        plane_marker = Marker()
        plane_marker.header.stamp = stamp
        plane_marker.header.frame_id = self.frame_id
        plane_marker.ns = 'marker_reco'
        plane_marker.id = 0
        plane_marker.type = Marker.TRIANGLE_LIST
        plane_marker.action = Marker.ADD
        plane_marker.pose.orientation.w = 1.0
        plane_marker.scale.x = 1.0
        plane_marker.scale.y = 1.0
        plane_marker.scale.z = 1.0
        plane_marker.color.r = 0.1
        plane_marker.color.g = 0.8
        plane_marker.color.b = 0.9
        plane_marker.color.a = 0.35

        triangles = []
        if len(points_3d) >= 3:
            for idx in range(1, len(points_3d) - 1):
                triangles.append((points_3d[0], points_3d[idx], points_3d[idx + 1]))
        for triangle in triangles:
            for vertex in triangle:
                pt = Point()
                pt.x = float(vertex[0])
                pt.y = float(vertex[1])
                pt.z = float(vertex[2])
                plane_marker.points.append(pt)

        q_marker = self._build_arrow_marker(1, stamp, center, q_end, (1.0, 0.2, 0.1))
        p_marker = self._build_arrow_marker(2, stamp, center, p_end, (0.1, 1.0, 0.1))
        up_marker = self._build_arrow_marker(3, stamp, center, up_end, (0.1, 0.4, 1.0))

        marker_array.markers.append(plane_marker)
        marker_array.markers.append(q_marker)
        marker_array.markers.append(p_marker)
        marker_array.markers.append(up_marker)
        self.viz_pub.publish(marker_array)

    def timer_callback(self):
        self._debug_frame_counter += 1

        ok, image_mat, color_frame, depth_frame = read_realsense_color_frame(
            self.pipeline,
            align=self.align,
        )

        if not ok:
            self.get_logger().warn('Failed to read frame from camera')
            return

        sharpened_image = sharpen_image(image_mat, sigma=10.0)
        white_range = shared_color_ranges['white'][0]
        hsv_masked_image = hsv_mask(sharpened_image, lower_hsv=white_range[0], upper_hsv=white_range[1])

        masked_gray = cv2.cvtColor(hsv_masked_image, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(masked_gray, 1, 255, cv2.THRESH_BINARY)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, self.kernel, iterations=2)

        hexagons = find_hexagon_contours(binary, min_area=1000, epsilon_ratio=0.02)
        roi_box = None
        if self.obb_model is not None:
            roi_box = predict_marker_roi(
                self.obb_model,
                image_mat,
                conf_thr=self.obb_conf_thr,
                iou_thr=self.obb_iou_thr,
                imgsz=self.obb_imgsz,
                roi_expand_ratio=self.roi_expand_ratio,
            )
            hexagons = filter_hexagons_in_roi(hexagons, roi_box)

        display_image = draw_hexagons(image_mat, hexagons)
        display_image = draw_roi_box(display_image, roi_box)

        #selected_hexagons = order_hexagons_clockwise_from_topleft(hexagons)
        hex_semantic = order_hexagons_by_semantic_position(hexagons, roi_box)
        valid_hex_count = sum(1 for item in hex_semantic if item is not None)

        plane_center_px = None
        if valid_hex_count == 3 or valid_hex_count == 4:
            points_3d, center_3d, q_axis, axes = estimate_plane_and_axes_from_hexagons(
                hex_semantic,
                self.rs_module,
                color_frame,
                depth_frame,
            )

            if points_3d is not None and center_3d is not None and q_axis is not None and axes is not None:
                plane_center_px = project_point_to_pixel(self.rs_module, color_frame, center_3d)
                p_axis, up_axis = axes
                self.publish_plane_geometry(points_3d, center_3d, q_axis, p_axis=p_axis, up_axis=up_axis)
                
                if (self._debug_frame_counter % self.debug_log_every_n_frames) == 0:
                    self.get_logger().info(
                        f'hex={len(hex_semantic)} center3d={np.round(center_3d, 4).tolist()} '
                        f'q={np.round(q_axis, 4).tolist()} '
                        f'p={np.round(p_axis, 4).tolist()} up={np.round(up_axis, 4).tolist()}'
                    )
            else:
                if (self._debug_frame_counter % self.debug_log_every_n_frames) == 0:
                    self.get_logger().warn('Center/axes estimation is unavailable for this frame; skip publish')

        display_image = draw_hexagon_plane_overlay(display_image, hex_semantic, plane_center_px=plane_center_px)

        if self.show_debug_windows:
            cv2.imshow('Marker Reco Debug', display_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()

    def destroy_node(self):
        if self.pipeline is not None:
            self.pipeline.stop()
        if self.show_debug_windows:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    print(f"[INFO] Python executable: {sys.executable}")
    rclpy.init(args=args)

    node = None
    try:
        node = MarkerRecoNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"[ERROR] Failed to start marker_reco_node: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
