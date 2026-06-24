#Grace's version of recognizing engineer marker light and calculate plane normal vector by using realsense camera.

import cv2
import numpy as np
import os
import sys
import shutil
import subprocess
import importlib
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped, PolygonStamped, Point32, Point
from visualization_msgs.msg import Marker, MarkerArray

rs = None

# 4x4 homogeneous transform from camera_color_optical_frame to base_link.
# Replace this matrix with your calibrated extrinsic parameters.
CAMERA_TO_BASELINK_T = np.array([
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
], dtype=np.float32)

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


def detect_corners(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners = cv2.goodFeaturesToTrack(gray, maxCorners=1000, qualityLevel=0.01, minDistance=10)
    return corners


def annotate_corners(img, corners):
    if corners is not None:
        corners = corners.astype(np.intp)
        for i in corners:
            x, y = i.ravel()
            cv2.circle(img, (x, y), 3, (0, 255, 0), -1)
    return img


def find_closed_polygon_contours(binary_img, min_area=500, epsilon_ratio=0.02):
    contours, _ = cv2.findContours(binary_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    quadrilaterals = []
    hexagons = []

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue

        perimeter = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon_ratio * perimeter, True)

        # Closed + 4 vertices: allow both convex and non-convex quadrilaterals.
        if len(approx) == 4:
            quadrilaterals.append((area, approx))
        elif len(approx) == 6:
            hexagons.append((area, approx))

    quadrilaterals.sort(key=lambda x: x[0], reverse=True)
    hexagons.sort(key=lambda x: x[0], reverse=True)
    return quadrilaterals, hexagons

def draw_quadrilaterals_and_hexagons(img, quadrilaterals, hexagons):
    out = img.copy()

    for i, (_, quadrilateral) in enumerate(quadrilaterals, start=1):
        poly = quadrilateral.astype(np.int32)
        cv2.polylines(out, [poly], True, (0, 255, 0), 2)

        moments = cv2.moments(poly)
        if moments["m00"] != 0:
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])
            cv2.putText(out, f"Q{i}", (cx - 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)

    for i, (_, hexagon) in enumerate(hexagons, start=1):
        poly = hexagon.astype(np.int32)
        cv2.polylines(out, [poly], True, (255, 255, 0), 2)

        moments = cv2.moments(poly)
        if moments["m00"] != 0:
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])
            cv2.putText(out, f"H{i}", (cx - 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 0, 0), 2)

    return out


def order_quad_points(quad_pts):
    pts = quad_pts.reshape(4, 2).astype(np.float32)
    ordered = np.zeros((4, 2), dtype=np.float32)

    s = pts.sum(axis=1)
    ordered[0] = pts[np.argmin(s)]  # top-left
    ordered[2] = pts[np.argmax(s)]  # bottom-right

    d = np.diff(pts, axis=1)
    ordered[1] = pts[np.argmin(d)]  # top-right
    ordered[3] = pts[np.argmax(d)]  # bottom-left

    return ordered


def line_intersection(p1, p2, p3, p4):
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    x4, y4 = p4

    den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(den) < 1e-6:
        return None

    num_x = (x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)
    num_y = (x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)
    return np.array([num_x / den, num_y / den], dtype=np.float32)


def estimate_plane_quad_from_hexagons(hexagons, expected_count=4, epsilon_ratio=0.02):
    if len(hexagons) < 1:
        return None, []

    selected = hexagons[:expected_count]
    all_pts = np.vstack([poly.reshape(-1, 2) for _, poly in selected]).astype(np.float32)

    hull = cv2.convexHull(all_pts.reshape(-1, 1, 2))
    perimeter = cv2.arcLength(hull, True)
    approx = cv2.approxPolyDP(hull, epsilon_ratio * perimeter, True)

    if len(approx) == 4:
        quad = approx.reshape(4, 2).astype(np.float32)
    else:
        # Fallback: robust outer 4 corners from min-area rectangle.
        rect = cv2.minAreaRect(all_pts)
        quad = cv2.boxPoints(rect).astype(np.float32)

    ordered = order_quad_points(quad)
    return ordered, selected


def draw_plane_diagonals(img, ordered_quad):
    out = img.copy()
    poly = ordered_quad.astype(np.int32).reshape(-1, 1, 2)

    # Draw estimated plane boundary from hexagon vertices.
    cv2.polylines(out, [poly], True, (0, 165, 255), 2)

    p0, p1, p2, p3 = ordered_quad

    # Two diagonals: p0-p2 and p1-p3.
    cv2.line(out, tuple(p0.astype(np.int32)), tuple(p2.astype(np.int32)), (255, 0, 255), 2)
    cv2.line(out, tuple(p1.astype(np.int32)), tuple(p3.astype(np.int32)), (0, 255, 255), 2)

    center = line_intersection(p0, p2, p1, p3)
    if center is not None:
        cx, cy = center.astype(np.int32)
        cv2.circle(out, (int(cx), int(cy)), 5, (0, 0, 255), -1)
        cv2.putText(out, "Center", (int(cx) + 6, int(cy) - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    return out, center


def deproject_quad_corners_to_3d(rs_module, color_frame, depth_frame, ordered_quad):
    if rs_module is None or color_frame is None or depth_frame is None:
        return None

    try:
        intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
    except Exception as exc:
        print(f"[WARN] Failed to read color intrinsics: {exc}")
        return None

    frame_width = color_frame.get_width()
    frame_height = color_frame.get_height()

    points_3d = []
    for corner in ordered_quad:
        u, v = int(round(float(corner[0]))), int(round(float(corner[1])))
        clipped_u = min(max(u, 0), frame_width - 1)
        clipped_v = min(max(v, 0), frame_height - 1)

        if clipped_u != u or clipped_v != v:
            print(
                f"[WARN] Clipped plane corner from ({u}, {v}) to ({clipped_u}, {clipped_v}) "
                f"for frame {frame_width}x{frame_height}"
            )

        u, v = clipped_u, clipped_v
        depth = depth_frame.get_distance(u, v)
        if depth <= 0.0:
            return None

        point = rs_module.rs2_deproject_pixel_to_point(intrinsics, [float(u), float(v)], float(depth))
        points_3d.append(np.asarray(point, dtype=np.float32))

    return np.asarray(points_3d, dtype=np.float32)


def compute_diagonal_vectors_and_normal(points_3d):
    if points_3d is None or len(points_3d) != 4:
        return None, None, None

    p0, p1, p2, p3 = points_3d
    diagonal_1 = p2 - p0
    diagonal_2 = p3 - p1

    normal = np.cross(diagonal_1, diagonal_2)
    norm = np.linalg.norm(normal)
    if norm < 1e-8:
        return diagonal_1, diagonal_2, None

    normal = normal / norm
    return diagonal_1, diagonal_2, normal


def compute_p_v_axes(points_3d):
    if points_3d is None or len(points_3d) != 4:
        return None, None

    p0, p1, p2, p3 = points_3d
    horizontal_1 = p1 - p0
    horizontal_2 = p2 - p3
    vertical_1 = p3 - p0
    vertical_2 = p2 - p1

    p_axis = horizontal_1 + horizontal_2
    v_axis = vertical_1 + vertical_2

    p_norm = np.linalg.norm(p_axis)
    v_norm = np.linalg.norm(v_axis)
    if p_norm < 1e-8 or v_norm < 1e-8:
        return None, None

    return p_axis / p_norm, v_axis / v_norm


def transform_points_and_normal_to_baselink(points_3d, normal):
    if points_3d is None or len(points_3d) != 4 or normal is None:
        return None, None

    transform = CAMERA_TO_BASELINK_T
    rotation = transform[:3, :3]
    translation = transform[:3, 3]

    transformed_points = np.asarray([
        (rotation @ point) + translation for point in points_3d
    ], dtype=np.float32)

    transformed_normal = rotation @ normal
    norm = np.linalg.norm(transformed_normal)
    if norm < 1e-8:
        return None, None
    transformed_normal = transformed_normal / norm

    return transformed_points, transformed_normal


def print_plane_geometry(points_3d, diagonal_1, diagonal_2, normal):
    print("[PLANE] 3D corners:")
    for index, point in enumerate(points_3d, start=1):
        print(f"[PLANE]   P{index}: {point.tolist()}")

    print(f"[PLANE] diagonal_1: {diagonal_1.tolist()}")
    print(f"[PLANE] diagonal_2: {diagonal_2.tolist()}")

    if normal is None:
        print("[PLANE] normal: unable to compute (degenerate diagonals)")
    else:
        print(f"[PLANE] normal: {normal.tolist()}")


class MarkerRecoNode(Node):
    def __init__(self):
        super().__init__('marker_reco_node')

        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('normal_topic', '/marker_reco/normal_direction')
        self.declare_parameter('p_topic', '/marker_reco/p_direction')
        self.declare_parameter('v_topic', '/marker_reco/v_direction')
        self.declare_parameter('corners_topic', '/marker_reco/corners_3d')
        self.declare_parameter('viz_topic', '/marker_reco/visualization')
        self.declare_parameter('normal_arrow_length', 0.12)
        self.declare_parameter('axis_arrow_length', 0.10)
        self.declare_parameter('show_debug_windows', True)
        self.declare_parameter('timer_period_sec', 0.03)

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.normal_topic = self.get_parameter('normal_topic').get_parameter_value().string_value
        self.p_topic = self.get_parameter('p_topic').get_parameter_value().string_value
        self.v_topic = self.get_parameter('v_topic').get_parameter_value().string_value
        self.corners_topic = self.get_parameter('corners_topic').get_parameter_value().string_value
        self.viz_topic = self.get_parameter('viz_topic').get_parameter_value().string_value
        self.normal_arrow_length = self.get_parameter('normal_arrow_length').get_parameter_value().double_value
        self.axis_arrow_length = self.get_parameter('axis_arrow_length').get_parameter_value().double_value
        self.show_debug_windows = self.get_parameter('show_debug_windows').get_parameter_value().bool_value
        self.timer_period_sec = self.get_parameter('timer_period_sec').get_parameter_value().double_value

        self.normal_pub = self.create_publisher(Vector3Stamped, self.normal_topic, 10)
        self.p_pub = self.create_publisher(Vector3Stamped, self.p_topic, 10)
        self.v_pub = self.create_publisher(Vector3Stamped, self.v_topic, 10)
        self.corners_pub = self.create_publisher(PolygonStamped, self.corners_topic, 10)
        self.viz_pub = self.create_publisher(MarkerArray, self.viz_topic, 10)

        self.rs_module = import_pyrealsense2()
        self.pipeline = open_realsense_pipeline()
        self.align = None

        if self.pipeline is None:
            raise RuntimeError('RealSense L515 is required, but no stream could be started.')

        if self.rs_module is not None:
            self.align = self.rs_module.align(self.rs_module.stream.color)

        self.kernel = np.ones((3, 3), np.uint8)
        self.timer = self.create_timer(self.timer_period_sec, self.timer_callback)
        self.get_logger().info(
            f'marker_reco_node started, publish frame {self.frame_id}, '
            f'normal to {self.normal_topic}, corners to {self.corners_topic}, viz to {self.viz_topic}'
        )

    def publish_plane_geometry(self, points_3d, normal):
        points_3d, normal = transform_points_and_normal_to_baselink(points_3d, normal)
        if points_3d is None or normal is None:
            self.get_logger().warn('Failed to transform plane geometry to base_link')
            return

        p_axis, v_axis = compute_p_v_axes(points_3d)
        if p_axis is None or v_axis is None:
            self.get_logger().warn('Failed to compute P/V axes for plane geometry')
            return

        stamp = self.get_clock().now().to_msg()

        normal_msg = Vector3Stamped()
        normal_msg.header.stamp = stamp
        normal_msg.header.frame_id = self.frame_id
        normal_msg.vector.x = float(normal[0])
        normal_msg.vector.y = float(normal[1])
        normal_msg.vector.z = float(normal[2])
        self.normal_pub.publish(normal_msg)

        p_msg = Vector3Stamped()
        p_msg.header.stamp = stamp
        p_msg.header.frame_id = self.frame_id
        p_msg.vector.x = float(p_axis[0])
        p_msg.vector.y = float(p_axis[1])
        p_msg.vector.z = float(p_axis[2])
        self.p_pub.publish(p_msg)

        v_msg = Vector3Stamped()
        v_msg.header.stamp = stamp
        v_msg.header.frame_id = self.frame_id
        v_msg.vector.x = float(v_axis[0])
        v_msg.vector.y = float(v_axis[1])
        v_msg.vector.z = float(v_axis[2])
        self.v_pub.publish(v_msg)

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

        self.publish_visualization_markers(points_3d, normal, p_axis, v_axis, stamp)

    def publish_visualization_markers(self, points_3d, normal, p_axis, v_axis, stamp):
        center = np.mean(points_3d, axis=0)
        arrow_end = center + (normal * float(self.normal_arrow_length))
        p_end = center + (p_axis * float(self.axis_arrow_length))
        v_end = center + (v_axis * float(self.axis_arrow_length))

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

        triangles = [
            (points_3d[0], points_3d[1], points_3d[2]),
            (points_3d[0], points_3d[2], points_3d[3]),
        ]
        for triangle in triangles:
            for vertex in triangle:
                pt = Point()
                pt.x = float(vertex[0])
                pt.y = float(vertex[1])
                pt.z = float(vertex[2])
                plane_marker.points.append(pt)

        arrow_marker = Marker()
        arrow_marker.header.stamp = stamp
        arrow_marker.header.frame_id = self.frame_id
        arrow_marker.ns = 'marker_reco'
        arrow_marker.id = 1
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD
        arrow_marker.pose.orientation.w = 1.0
        arrow_marker.scale.x = 0.01
        arrow_marker.scale.y = 0.02
        arrow_marker.scale.z = 0.03
        arrow_marker.color.r = 1.0
        arrow_marker.color.g = 0.2
        arrow_marker.color.b = 0.1
        arrow_marker.color.a = 1.0

        start_pt = Point()
        start_pt.x = float(center[0])
        start_pt.y = float(center[1])
        start_pt.z = float(center[2])

        end_pt = Point()
        end_pt.x = float(arrow_end[0])
        end_pt.y = float(arrow_end[1])
        end_pt.z = float(arrow_end[2])

        arrow_marker.points = [start_pt, end_pt]

        p_marker = Marker()
        p_marker.header.stamp = stamp
        p_marker.header.frame_id = self.frame_id
        p_marker.ns = 'marker_reco'
        p_marker.id = 2
        p_marker.type = Marker.ARROW
        p_marker.action = Marker.ADD
        p_marker.pose.orientation.w = 1.0
        p_marker.scale.x = 0.01
        p_marker.scale.y = 0.02
        p_marker.scale.z = 0.03
        p_marker.color.r = 0.1
        p_marker.color.g = 1.0
        p_marker.color.b = 0.1
        p_marker.color.a = 1.0

        p_start_pt = Point()
        p_start_pt.x = float(center[0])
        p_start_pt.y = float(center[1])
        p_start_pt.z = float(center[2])

        p_end_pt = Point()
        p_end_pt.x = float(p_end[0])
        p_end_pt.y = float(p_end[1])
        p_end_pt.z = float(p_end[2])

        p_marker.points = [p_start_pt, p_end_pt]

        v_marker = Marker()
        v_marker.header.stamp = stamp
        v_marker.header.frame_id = self.frame_id
        v_marker.ns = 'marker_reco'
        v_marker.id = 3
        v_marker.type = Marker.ARROW
        v_marker.action = Marker.ADD
        v_marker.pose.orientation.w = 1.0
        v_marker.scale.x = 0.01
        v_marker.scale.y = 0.02
        v_marker.scale.z = 0.03
        v_marker.color.r = 0.1
        v_marker.color.g = 0.4
        v_marker.color.b = 1.0
        v_marker.color.a = 1.0

        v_start_pt = Point()
        v_start_pt.x = float(center[0])
        v_start_pt.y = float(center[1])
        v_start_pt.z = float(center[2])

        v_end_pt = Point()
        v_end_pt.x = float(v_end[0])
        v_end_pt.y = float(v_end[1])
        v_end_pt.z = float(v_end[2])

        v_marker.points = [v_start_pt, v_end_pt]

        marker_array.markers.append(plane_marker)
        marker_array.markers.append(arrow_marker)
        marker_array.markers.append(p_marker)
        marker_array.markers.append(v_marker)
        self.viz_pub.publish(marker_array)

    def timer_callback(self):
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

        quadrilaterals, hexagons = find_closed_polygon_contours(binary, min_area=1000, epsilon_ratio=0.02)
        polygon_image = draw_quadrilaterals_and_hexagons(image_mat, quadrilaterals, hexagons)

        plane_quad, selected_hexagons = estimate_plane_quad_from_hexagons(
            hexagons,
            expected_count=4,
            epsilon_ratio=0.02,
        )

        display_image = polygon_image
        if plane_quad is not None and len(selected_hexagons) >= 2:
            diagonal_image, _ = draw_plane_diagonals(polygon_image, plane_quad)
            display_image = diagonal_image

            points_3d = deproject_quad_corners_to_3d(self.rs_module, color_frame, depth_frame, plane_quad)
            if points_3d is not None:
                diagonal_1, diagonal_2, normal = compute_diagonal_vectors_and_normal(points_3d)
                if diagonal_1 is not None and diagonal_2 is not None and normal is not None:
                    self.publish_plane_geometry(points_3d, normal)

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


def open_camera_with_fallback(indices, backends):
    attempts = []
    for idx in indices:
        for backend_name, backend_flag in backends:
            if backend_flag is None:
                cap = cv2.VideoCapture(idx)
            else:
                cap = cv2.VideoCapture(idx, backend_flag)

            if cap.isOpened():
                return cap, idx, backend_name, attempts

            cap.release()
            attempts.append(f"index={idx}, backend={backend_name}")

    return None, None, None, attempts


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
