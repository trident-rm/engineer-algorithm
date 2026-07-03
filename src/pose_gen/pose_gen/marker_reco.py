#Grace's version of recognizing engineer marker light and calculate plane q vector by using realsense camera.

import cv2
import numpy as np
import os
import sys
import shutil
import subprocess
import importlib
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Vector3Stamped, PolygonStamped, Point32, Point
from visualization_msgs.msg import Marker, MarkerArray

rs = None
ultralytics_module = None

# 4x4 homogeneous transform from camera_color_optical_frame to base_link.
# Replace this matrix with your calibrated extrinsic parameters.
CAMERA_TO_BASELINK_T = np.array([
    [0.0, 0.0, 1.0, -0.05],
    [0.0, 1.0, 0.0, 0.0],
    [-1.0, 0.0, 0.0, 0.4],
    [0.0, 0.0, 0.0, 1.0],
], dtype=np.float32)


camera_marker = Marker()
camera_marker.header.frame_id = 'base_link'
camera_marker.ns = 'marker_reco'
camera_marker.id = 0
camera_marker.type = Marker.SPHERE
camera_marker.action = Marker.ADD
camera_marker.pose.orientation.w = 1.0
camera_marker.pose.position.x = 0.05
camera_marker.pose.position.y = 0.0
camera_marker.pose.position.z = 0.4
camera_marker.scale.x = 0.05
camera_marker.scale.y = 0.05
camera_marker.scale.z = 0.05
camera_marker.color.r = 0.1
camera_marker.color.g = 0.8
camera_marker.color.b = 0.9
camera_marker.color.a = 0.35

# 初始化共享颜色范围（红色和蓝色）
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
        approx = cv2.approxPolyDP(contour, epsilon_ratio * perimeter, True)
        if len(approx) == 6:
            hexagons.append((area, approx))

    hexagons.sort(key=lambda x: x[0], reverse=True)
    return hexagons


def draw_hexagons(img, hexagons):
    out = img.copy()

    for i, (_, hexagon) in enumerate(hexagons, start=1):
        poly = hexagon.astype(np.int32)
        cv2.polylines(out, [poly], True, (255, 255, 0), 2)

        moments = cv2.moments(poly)
        if moments["m00"] != 0:
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])
            cv2.putText(out, f"H{i}", (cx - 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 0, 0), 2)

    return out


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
    ordered = np.roll(ordered, -start_idx, axis=0)
    order = np.roll(order, -start_idx, axis=0)
    return ordered, order


def order_hexagons_clockwise_from_topleft(hexagons):
    if hexagons is None or len(hexagons) == 0:
        return []

    centers = [_polygon_center_2d(h) for _, h in hexagons]
    _, order = _order_points_clockwise_from_topleft(centers)
    if order is None:
        return list(hexagons)
    return [hexagons[int(idx)] for idx in order]


def _extract_plane_corner_vertex_2d(hexagon, box_center_xy):
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


def filter_hexagons_in_roi(hexagons, roi_box_xyxy):
    if roi_box_xyxy is None:
        return hexagons

    x1, y1, x2, y2 = roi_box_xyxy
    filtered = []
    for area, hexagon in hexagons:
        pts = hexagon.reshape(-1, 2).astype(np.float32)
        center = np.mean(pts, axis=0)
        cx, cy = float(center[0]), float(center[1])
        if x1 <= cx <= x2 and y1 <= cy <= y2:
            filtered.append((area, hexagon))
    return filtered


def draw_roi_box(image, roi_box_xyxy):
    if roi_box_xyxy is None:
        return image
    out = image.copy()
    x1, y1, x2, y2 = roi_box_xyxy
    cv2.rectangle(out, (x1, y1), (x2, y2), (0, 128, 255), 2)
    cv2.putText(out, 'NN ROI', (x1 + 4, max(20, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 128, 255), 2)
    return out


def import_ultralytics():
    global ultralytics_module
    if ultralytics_module is None:
        ultralytics_module = importlib.import_module('ultralytics')
    return ultralytics_module


def patch_numpy_bitgenerator_unpickle_compat():
    """Bypass incompatible numpy RNG pickle payloads in legacy checkpoints.

    Some historical training checkpoints serialize numpy RNG internals that are
    version-sensitive and irrelevant for inference. We replace constructor hooks
    with lightweight placeholders so torch can load model weights.
    """
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


def load_obb_model(model_path):
    module = import_ultralytics()
    if module is None:
        print('[WARN] ultralytics is unavailable. ROI filtering disabled.')
        return None

    # Fix compatibility for some historical checkpoints serialized with
    # BitGenerator classes instead of string names.
    patch_numpy_bitgenerator_unpickle_compat()

    if not os.path.isfile(model_path):
        print(f'[WARN] OBB model file does not exist: {model_path}')
        return None

    try:
        model = module.YOLO(model_path)
    except Exception as exc:
        print(f'[WARN] Failed to load OBB model {model_path}: {exc}')
        return None

    print(f'[INFO] OBB model loaded: {model_path}')
    return model


def resolve_default_obb_model_path():
    env_path = os.getenv('OBB_MODEL_PATH')
    if env_path:
        return os.path.abspath(env_path)

    module_dir = os.path.abspath(os.path.dirname(__file__))

    # Candidate 1: installed python package data, e.g. .../site-packages/pose_gen/weights/hkust_best.pt
    candidates = [
        os.path.abspath(os.path.join(module_dir, 'weights', 'hkust_best.pt')),
        # Candidate 2: source layout, e.g. src/pose_gen/weights/hkust_best.pt
        os.path.abspath(os.path.join(module_dir, '..', 'weights', 'hkust_best.pt')),
    ]

    # Candidate 3: ROS install share path, e.g. <prefix>/share/pose_gen/weights/hkust_best.pt
    marker = '/lib/python'
    if marker in module_dir:
        install_prefix = module_dir.split(marker, 1)[0]
        candidates.append(os.path.abspath(os.path.join(install_prefix, 'share', 'pose_gen', 'weights', 'hkust_best.pt')))

    for path in candidates:
        if os.path.isfile(path):
            return path

    # Fallback keeps previous source-style default for clear warning output.
    return os.path.abspath(os.path.join(module_dir, '..', 'weights', 'hkust_best.pt'))


def _resolve_yolo_class_name(names, class_id):
    if names is None:
        return None
    try:
        if isinstance(names, dict):
            return names.get(int(class_id))
        if isinstance(names, (list, tuple)):
            idx = int(class_id)
            if 0 <= idx < len(names):
                return names[idx]
    except Exception:
        return None
    return None


def predict_marker_roi(model, image_mat, conf_thr=0.25, iou_thr=0.45, imgsz=640, roi_expand_ratio=0.20):
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

    # Simple debug: print classes from either boxes or obb.
    detected_classes = []
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

    # Preferred path: standard bbox outputs (hkust_best often uses boxes + keypoints).
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

    # Fallback path: OBB outputs.
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


def draw_hexagon_plane_overlay(img, selected_hexagons, plane_center_px=None):
    out = img.copy()
    if selected_hexagons is None or len(selected_hexagons) == 0:
        return out

    ordered_hex = order_hexagons_clockwise_from_topleft(selected_hexagons)

    # Compute an image-space box center to classify each marker as UL/UR/LL/LR.
    tmp_centers = np.asarray([_polygon_center_2d(h) for _, h in ordered_hex], dtype=np.float32)
    box_center = np.asarray([
        0.5 * (float(np.min(tmp_centers[:, 0])) + float(np.max(tmp_centers[:, 0]))),
        0.5 * (float(np.min(tmp_centers[:, 1])) + float(np.max(tmp_centers[:, 1]))),
    ], dtype=np.float32)

    def _quad_label(center_xy):
        left = center_xy[0] <= box_center[0]
        up = center_xy[1] <= box_center[1]
        if up and left:
            return 'UL'
        if up and (not left):
            return 'UR'
        if (not up) and left:
            return 'LL'
        return 'LR'

    centers = []
    labels = ['A', 'B', 'C', 'D']
    for idx, (_, hexagon) in enumerate(ordered_hex):
        center = _polygon_center_2d(hexagon)
        centers.append(center)
        cx, cy = int(center[0]), int(center[1])
        cv2.circle(out, (cx, cy), 4, (0, 200, 255), -1)
        seq_label = labels[idx] if idx < len(labels) else '?'
        q_label = _quad_label(center)
        cv2.putText(out, f'{q_label}({seq_label})', (cx + 5, cy + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 120), 1)

    # Visual reference for quadrant split center.
    bcx, bcy = int(box_center[0]), int(box_center[1])
    cv2.drawMarker(out, (bcx, bcy), (255, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=10, thickness=1)

    centers = np.asarray(centers, dtype=np.float32)
    plane_center = None
    if plane_center_px is not None:
        plane_center = np.asarray(plane_center_px, dtype=np.float32)
        label = 'PlaneCenter(3D diag)'
    elif len(centers) >= 3:
        plane_center = box_center
        label = 'PlaneCenter(2D box)'
    elif len(centers) >= 4:
        # A/B/C/D ordered clockwise from top-left.
        plane_center = 0.5 * (centers[0] + centers[2])
        label = 'PlaneCenter(2D)'

    if plane_center is not None:
        pcx, pcy = int(plane_center[0]), int(plane_center[1])
        cv2.circle(out, (pcx, pcy), 6, (0, 0, 255), -1)
        cv2.putText(out, label, (pcx + 6, pcy - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    if len(centers) >= 4:
        rel = centers - plane_center
        angles = np.arctan2(rel[:, 1], rel[:, 0])
        order = np.argsort(angles)
        ordered = centers[order][:4]

        poly = ordered.astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(out, [poly], True, (0, 165, 255), 2)
        cv2.line(out, tuple(ordered[0].astype(np.int32)), tuple(ordered[2].astype(np.int32)), (255, 0, 255), 2)
        cv2.line(out, tuple(ordered[1].astype(np.int32)), tuple(ordered[3].astype(np.int32)), (0, 255, 255), 2)

    return out



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
    if hexagons is None or len(hexagons) < 3:
        return None, None, None, None

    ordered_hex = order_hexagons_clockwise_from_topleft(hexagons)
    if len(ordered_hex) < 3:
        return None, None, None, None

    selected = ordered_hex[:4]
    centers_2d = [_polygon_center_2d(h) for _, h in selected]
    box_center_2d = np.asarray([
        0.5 * (float(np.min([c[0] for c in centers_2d])) + float(np.max([c[0] for c in centers_2d]))),
        0.5 * (float(np.min([c[1] for c in centers_2d])) + float(np.max([c[1] for c in centers_2d]))),
    ], dtype=np.float32)

    corner_map_3d = {}

    def _ab_label(center_xy):
        left = center_xy[0] <= box_center_2d[0]
        up = center_xy[1] <= box_center_2d[1]
        if up and left:
            return 'A'
        if up and (not left):
            return 'B'
        if (not up) and left:
            return 'D'
        return 'C'

    for _, hexagon in selected:
        center_xy = _polygon_center_2d(hexagon)
        label = _ab_label(center_xy)
        corner_px = _extract_plane_corner_vertex_2d(hexagon, box_center_2d)
        pt3 = deproject_pixel_to_3d(rs_module, color_frame, depth_frame, corner_px[0], corner_px[1])
        if pt3 is None:
            continue
        if label not in corner_map_3d:
            corner_map_3d[label] = pt3

    if len(corner_map_3d) < 3:
        return None, None, None, None

    labels = ['A', 'B', 'C', 'D']
    missing = [l for l in labels if l not in corner_map_3d]
    if len(missing) > 1:
        return None, None, None, None

    if len(missing) == 1:
        m = missing[0]
        if m == 'A' and all(k in corner_map_3d for k in ['B', 'C', 'D']):
            corner_map_3d['A'] = corner_map_3d['B'] + corner_map_3d['D'] - corner_map_3d['C']
        elif m == 'B' and all(k in corner_map_3d for k in ['A', 'C', 'D']):
            corner_map_3d['B'] = corner_map_3d['A'] + corner_map_3d['C'] - corner_map_3d['D']
        elif m == 'C' and all(k in corner_map_3d for k in ['A', 'B', 'D']):
            corner_map_3d['C'] = corner_map_3d['B'] + corner_map_3d['D'] - corner_map_3d['A']
        elif m == 'D' and all(k in corner_map_3d for k in ['A', 'B', 'C']):
            corner_map_3d['D'] = corner_map_3d['A'] + corner_map_3d['C'] - corner_map_3d['B']
        else:
            return None, None, None, None

    observed_points = np.asarray(list(corner_map_3d.values()), dtype=np.float32)
    if len(observed_points) < 3:
        return None, None, None, None

    points_3d = np.asarray([corner_map_3d[l] for l in labels], dtype=np.float32)

    _, q_axis = _fit_plane_svd(observed_points)
    if q_axis is None:
        return None, None, None, None

    center_3d = 0.5 * (points_3d[0] + points_3d[2])
    p_axis, up_axis = compute_p_up_axes(points_3d)
    if p_axis is None or up_axis is None:
        return None, None, None, None

    return points_3d, center_3d.astype(np.float32), q_axis.astype(np.float32), (p_axis.astype(np.float32), up_axis.astype(np.float32))


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


class MarkerRecoNode(Node):
    def __init__(self):
        super().__init__('marker_reco_node')

        self.frame_id = 'base_link'
        self.up_topic = '/marker_reco/up_direction'
        self.p_topic = '/marker_reco/p_direction'
        self.q_topic = '/marker_reco/q_direction'
        self.corners_topic = '/marker_reco/corners_3d'
        self.viz_topic = '/marker_reco/visualization'
        self.q_arrow_length = 0.12
        self.axis_arrow_length = 0.10
        self.show_debug_windows = True
        self.timer_period_sec = 0.1
        self.use_nn_roi = True
        self.obb_model_path = resolve_default_obb_model_path()
        self.obb_conf_thr = 0.25
        self.obb_iou_thr = 0.45
        self.obb_imgsz = 640
        self.roi_expand_ratio = 0.05
        self.debug_log_every_n_frames = 15

        self.up_pub = self.create_publisher(Vector3Stamped, self.up_topic, 10)
        self.p_pub = self.create_publisher(Vector3Stamped, self.p_topic, 10)
        self.q_pub = self.create_publisher(Vector3Stamped, self.q_topic, 10)
        self.camera_pub = self.create_publisher(Marker, '/marker_reco/camera', 10)
        self.corners_pub = self.create_publisher(PolygonStamped, self.corners_topic, 10)
        self.center_pub = self.create_publisher(PointStamped, '/marker_reco/center_3d', 10)
        self.viz_pub = self.create_publisher(MarkerArray, self.viz_topic, 10)

        self.rs_module = import_pyrealsense2()
        self.pipeline = open_realsense_pipeline()
        self.align = None

        if self.pipeline is None:
            raise RuntimeError('RealSense L515 is required, but no stream could be started.')

        if self.rs_module is not None:
            self.align = self.rs_module.align(self.rs_module.stream.color)

        self.obb_model = None
        if self.use_nn_roi:
            self.obb_model = load_obb_model(self.obb_model_path)
            if self.obb_model is None:
                self.get_logger().warn('NN ROI is enabled but model is unavailable; fallback to contour-only mode')

        self.kernel = np.ones((3, 3), np.uint8)
        self._debug_frame_counter = 0
        self.timer = self.create_timer(self.timer_period_sec, self.timer_callback)
        self.get_logger().info(
            f'marker_reco_node started, publish frame {self.frame_id}, '
            f'up to {self.up_topic}, q to {self.q_topic}, p to {self.p_topic}, '
            f'corners to {self.corners_topic}, viz to {self.viz_topic}'
        )

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

        # up: vertical edge direction on the detected plane.
        up_msg = Vector3Stamped()
        up_msg.header.stamp = stamp
        up_msg.header.frame_id = self.frame_id
        up_msg.vector.x = float(up_axis[0])
        up_msg.vector.y = float(up_axis[1])
        up_msg.vector.z = float(up_axis[2])
        self.up_pub.publish(up_msg)

        p_msg = Vector3Stamped()
        p_msg.header.stamp = stamp
        p_msg.header.frame_id = self.frame_id
        p_msg.vector.x = float(p_axis[0])
        p_msg.vector.y = float(p_axis[1])
        p_msg.vector.z = float(p_axis[2])
        self.p_pub.publish(p_msg)

        # q: plane normal direction.
        q_msg = Vector3Stamped()
        q_msg.header.stamp = stamp
        q_msg.header.frame_id = self.frame_id
        q_msg.vector.x = float(q_axis[0])
        q_msg.vector.y = float(q_axis[1])
        q_msg.vector.z = float(q_axis[2])
        self.q_pub.publish(q_msg)

        center_msg = PointStamped()
        center_msg.header.stamp = stamp
        center_msg.header.frame_id = self.frame_id
        center_msg.point.x = float(center_3d[0])
        center_msg.point.y = float(center_3d[1])
        center_msg.point.z = float(center_3d[2])
        self.center_pub.publish(center_msg)

        corners_msg = PolygonStamped()
        corners_msg.header.stamp = stamp
        corners_msg.header.frame_id = self.frame_id
        for point in points_3d:
            corner = Point32()
            corner.x = float(point[0])
            corner.y = float(point[1])
            corner.z = float(point[2])
            corners_msg.polygon.points.append(corner)
        self.corners_pub.publish(corners_msg)

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
        arrow_end = center + (q_axis * float(self.q_arrow_length))
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

        arrow_marker = self._build_arrow_marker(1, stamp, center, arrow_end, (1.0, 0.2, 0.1))
        p_marker = self._build_arrow_marker(2, stamp, center, p_end, (0.1, 1.0, 0.1))
        up_marker = self._build_arrow_marker(3, stamp, center, up_end, (0.1, 0.4, 1.0))

        marker_array.markers.append(plane_marker)
        marker_array.markers.append(arrow_marker)
        marker_array.markers.append(p_marker)
        marker_array.markers.append(up_marker)
        self.viz_pub.publish(marker_array)

    def timer_callback(self):
        camera_stamp = self.get_clock().now().to_msg()
        camera_marker.header.stamp = camera_stamp
        self.camera_pub.publish(camera_marker)  # Dummy message to indicate camera is alive
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

        polygon_image = draw_hexagons(image_mat, hexagons)
        polygon_image = draw_roi_box(polygon_image, roi_box)

        selected_hexagons = order_hexagons_clockwise_from_topleft(hexagons)

        display_image = polygon_image
        plane_center_px = None
        if len(selected_hexagons) >= 3:
            points_3d, center_3d, q_axis, axes = estimate_plane_and_axes_from_hexagons(
                selected_hexagons,
                self.rs_module,
                color_frame,
                depth_frame,
            )

            if points_3d is not None and center_3d is not None and q_axis is not None and axes is not None:
                plane_center_px = project_point_to_pixel(self.rs_module, color_frame, center_3d)
                p_axis, up_axis = axes
                self.publish_plane_geometry(points_3d, center_3d, q_axis, p_axis=p_axis, up_axis=up_axis)

                n = max(1, int(self.debug_log_every_n_frames))
                if (self._debug_frame_counter % n) == 0:
                    self.get_logger().info(
                        f'hex={len(selected_hexagons)} center3d={np.round(center_3d, 4).tolist()} '
                        f'q={np.round(q_axis, 4).tolist()} '
                        f'p={np.round(p_axis, 4).tolist()} up={np.round(up_axis, 4).tolist()}'
                    )
            else:
                n = max(1, int(self.debug_log_every_n_frames))
                if (self._debug_frame_counter % n) == 0:
                    self.get_logger().warn('Center/axes estimation is unavailable for this frame; skip publish')

        display_image = draw_hexagon_plane_overlay(display_image, selected_hexagons, plane_center_px=plane_center_px)

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


def parse_int_env(name, default):
    value = os.getenv(name)
    if value is None:
        return default
    try:
        return int(value)
    except ValueError:
        print(f"[WARN] Invalid {name}={value}, fallback to {default}")
        return default


def import_pyrealsense2():
    global rs
    if rs is not None:
        return rs

    try:
        rs = importlib.import_module("pyrealsense2")
    except Exception as exc:
        print(f"[WARN] pyrealsense2 import failed: {exc}")
        rs = None

    return rs


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
    rs_module = import_pyrealsense2()
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
        print("[ERROR] Troubleshooting:")
        print("[ERROR] 1) Replug L515 to a USB 3.0 port and run rs-enumerate-devices")
        print("[ERROR] 2) Verify pyrealsense2 and librealsense versions are compatible")
        print("[ERROR] 3) On Linux, verify device permissions/udev rules")
        print("[ERROR] 4) If multiple devices are connected, set REALSENSE_SERIAL")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
