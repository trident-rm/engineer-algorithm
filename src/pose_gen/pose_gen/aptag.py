"""
Detector Placeholder.
Using Apriltag for detecting corners.
"""

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

import time
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge
import apriltag

import json
import os

class AptagNode(Node):
    def __init__(self, tag_size=0.1):
        super().__init__('aptag_node')
        self.fps_test_pub = self.create_publisher(String, '/fps_test_aptag', 10)
        self.corner_pub = self.create_publisher(String, '/corner_aptag', 10)
        self.cam_sub = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)

        self.aptag_detector = apriltag.Detector(
            apriltag.DetectorOptions(families="tag36h11")
        )
        self.bridge = CvBridge()
        self.img = None
        self.cam_matrix = np.zeros((3, 3), dtype=np.float32)
        self.dist_coeffs = np.zeros((5,), dtype=np.float32)
        self.tag_size = tag_size  # Physical size of AprilTag in meters

    def load_camera_params(self, cam_matrix, dist_coeffs):
        """
        Load camera parameters for undistortion.
        
        Args:
            cam_matrix: Camera intrinsic matrix.
            dist_coeffs: Distortion coefficients.
        """
        self.cam_matrix = np.array(cam_matrix, dtype=np.float32)
        self.dist_coeffs = np.array(dist_coeffs, dtype=np.float32)
        self.get_logger().info("Camera parameters loaded successfully.")

    def image_callback(self, msg):
        try:
            # time1 = time.time()
            self.img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            # time2 = time.time()
            # self.get_logger().info(f"Image conversion took {time2 - time1:.4f} seconds")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

    def publish_corners(self, corners, plane_center):
        """
        Publish the detected corners as a JSON string.
        
        Args:
            corners: List of corners detected by the AprilTag detector.
        """
        # Ensure corners are published in order of [lb, rb, rt, lt]
        ordered_corners = None
        lb = rb = rt = lt = None
        if len(corners) == 4:
            # if higher and lefter than the center, then lt
            # if higher and righter than the center, then rt
            # if just on top of the center, then lt
            for corner in corners:
                if corner[0] <= plane_center[0] and corner[1] <= plane_center[1]:
                    lb = corner
                elif corner[0] >= plane_center[0] and corner[1] <= plane_center[1]:
                    rb = corner
                elif corner[0] >= plane_center[0] and corner[1] >= plane_center[1]:
                    rt = corner
                elif corner[0] <= plane_center[0] and corner[1] >= plane_center[1]:
                    lt = corner
            if lb is None or rb is None or rt is None or lt is None:
                self.get_logger().warn("Not all corners detected, cannot publish.")
                self.get_logger().warn(f'lb: {lb}, rb: {rb}, rt: {rt}, lt: {lt}')
                return
            ordered_corners = np.array([lb, rb, rt, lt])
        else:
            self.get_logger().warn(f"Unexpected number of corners: {len(corners)}")
            return

        self.get_logger().info(f"Publishing corners: {ordered_corners}")
        corners_json = json.dumps(ordered_corners.tolist())
        self.get_logger().info(f"Corners JSON: {corners_json}")
        self.corner_pub.publish(String(data=corners_json))

    def aptag_detection(self, image):
        all_ids = []
        all_corners = []
        all_centers = []
        all_margins = []
        all_ctr_3d_pts = []
        all_corners_3d = []
        
        gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        result = self.aptag_detector.detect(gray_image)
        display_image = image.copy()
        
        if len(result) > 0:
            print(f'=='*50)
            print(f"Detected {len(result)} AprilTags:")
            
            # First pass: collect all 2D data
            all_corners_2d_flat = []
            for i, tag in enumerate(result):
                # Support both legacy dict output and Detection object output.
                if isinstance(tag, dict):
                    tag_id = tag['id']
                    center = np.array(tag['center'], dtype=np.float32)
                    corners = np.array(tag['lb-rb-rt-lt'], dtype=np.float32)
                    hamming = tag['hamming']
                    margin = tag['margin']
                else:
                    tag_id = int(tag.tag_id)
                    center = np.array(tag.center, dtype=np.float32)
                    corners = np.array(tag.corners, dtype=np.float32)
                    hamming = int(getattr(tag, 'hamming', -1))
                    margin = float(getattr(tag, 'decision_margin', 0.0))

                all_ids.append(tag_id)
                all_corners.append(corners)
                all_centers.append(center)
                all_margins.append(margin)
                
                # Flatten corners for depth estimation
                for corner in corners:
                    all_corners_2d_flat.append(corner)

                print(f"Tag {i+1}:")
                print(f"  ID: {tag_id}")
                print(f"  Center: ({center[0]:.2f}, {center[1]:.2f})")
                print(f"  Hamming distance: {hamming}")
                print(f"  Margin: {margin:.2f}")
                print(f"  Corners: {corners}")
            
            # Estimate depth using coplanar constraint
            if len(all_corners_2d_flat) >= 8:  # Use advanced method with 2+ tags
                estimated_depth = self.estimate_plane_depth_advanced(all_corners_2d_flat, self.tag_size)
            else:
                estimated_depth = self.estimate_plane_depth(all_corners_2d_flat, self.tag_size)
            print(f"Estimated depth of coplanar surface: {estimated_depth:.3f}m")
            
            # Second pass: calculate 3D points with estimated depth
            for i, (center, corners) in enumerate(zip(all_centers, all_corners)):
                # Calculate 3D center point with estimated depth
                ctr_pt_3d = self.get_3d_point(center, estimated_depth)
                all_ctr_3d_pts.append(ctr_pt_3d)
                
                # Calculate 3D corners with estimated depth
                corners_3d = self.get_corners_3d_with_depth(corners, estimated_depth)
                all_corners_3d.append(corners_3d)
                
                print(f"Tag {i+1} 3D Info:")
                print(f"  Center 3D: ({ctr_pt_3d[0]:.3f}, {ctr_pt_3d[1]:.3f}, {ctr_pt_3d[2]:.3f})")
                print(f"  Corners 3D: {[f'({c[0]:.3f}, {c[1]:.3f}, {c[2]:.3f})' for c in corners_3d]}")
                
            plane_center = self.get_center(all_centers)
            plane_center_3d = self.get_3d_point(plane_center, estimated_depth)
        else:
            # self.get_logger().info("No tags detected")
            return None

        return_info = {
            'detected_tags': len(result),
            'display_image': display_image,
            'timestamp': time.time(),
            'ids': all_ids,
            'corners': all_corners,
            'centers': all_centers,
            'margins': all_margins,
            'ctr_3d_points': all_ctr_3d_pts,
            'corners_3d': all_corners_3d,
            'plane_center': plane_center,
            'plane_center_3d': plane_center_3d,
            'estimated_depth': estimated_depth,
        }
        return return_info

    def get_center(self, corners):
        return np.mean(corners, axis=0)

    def get_3d_point(self, point, depth=1.0):
        """
        Convert 2D point to 3D point using loaded camera parameters and given depth.
        """
        # Use actual camera parameters from loaded camera matrix
        fx = self.cam_matrix[0, 0]  # Focal length in x
        fy = self.cam_matrix[1, 1]  # Focal length in y
        cx = self.cam_matrix[0, 2]  # Principal point x
        cy = self.cam_matrix[1, 2]  # Principal point y
        
        x = (point[0] - cx) / fx * depth
        y = (point[1] - cy) / fy * depth
        return np.array([x, y, depth])

    def estimate_plane_depth(self, corners_2d, tag_size=0.1):
        """
        Estimate the depth of a coplanar surface using AprilTag corners.
        
        Args:
            corners_2d: List of 2D corner points from all detected tags
            tag_size: Physical size of AprilTag in meters (default: 10cm)
            
        Returns:
            estimated_depth: Average depth of the plane in meters
        """
        if len(corners_2d) < 4:
            self.get_logger().warn("Need at least 4 corners to estimate plane depth")
            return 1.0
        
        # Camera parameters
        fx = self.cam_matrix[0, 0]
        fy = self.cam_matrix[1, 1]
        cx = self.cam_matrix[0, 2]
        cy = self.cam_matrix[1, 2]
        
        depth_estimates = []
        
        # For each tag, estimate depth using the known physical size
        for i in range(0, len(corners_2d), 4):  # Process 4 corners at a time (one tag)
            if i + 3 < len(corners_2d):
                tag_corners = corners_2d[i:i+4]
                
                # Calculate pixel distances between adjacent corners
                pixel_distances = []
                for j in range(4):
                    p1 = tag_corners[j]
                    p2 = tag_corners[(j + 1) % 4]
                    pixel_dist = np.linalg.norm(p2 - p1)
                    pixel_distances.append(pixel_dist)
                
                # Average pixel distance for this tag
                avg_pixel_dist = np.mean(pixel_distances)
                
                # Estimate depth using pinhole camera model
                # depth = (focal_length * real_world_size) / pixel_size
                # Use average of fx and fy for focal length
                avg_focal_length = (fx + fy) / 2
                estimated_depth = (avg_focal_length * tag_size) / avg_pixel_dist
                depth_estimates.append(estimated_depth)
        
        if depth_estimates:
            final_depth = np.mean(depth_estimates)
            self.get_logger().info(f"Estimated plane depth: {final_depth:.3f}m from {len(depth_estimates)} tags")
            return final_depth
        else:
            self.get_logger().warn("Could not estimate depth, using default value")
            return 1.0

    def estimate_plane_depth_advanced(self, corners_2d, tag_size=0.1):
        """
        Advanced depth estimation using plane fitting for better accuracy with multiple tags.
        
        Args:
            corners_2d: List of 2D corner points from all detected tags
            tag_size: Physical size of AprilTag in meters
            
        Returns:
            estimated_depth: Refined depth estimate using plane fitting
        """
        if len(corners_2d) < 8:  # Need at least 2 tags for plane fitting
            return self.estimate_plane_depth(corners_2d, tag_size)
        
        # Get initial depth estimate
        initial_depth = self.estimate_plane_depth(corners_2d, tag_size)
        
        # Convert all 2D points to 3D using initial depth
        points_3d = []
        fx = self.cam_matrix[0, 0]
        fy = self.cam_matrix[1, 1]
        cx = self.cam_matrix[0, 2]
        cy = self.cam_matrix[1, 2]
        
        for corner_2d in corners_2d:
            x = (corner_2d[0] - cx) / fx * initial_depth
            y = (corner_2d[1] - cy) / fy * initial_depth
            points_3d.append([x, y, initial_depth])
        
        points_3d = np.array(points_3d)
        
        # Fit a plane to the 3D points using SVD
        centroid = np.mean(points_3d, axis=0)
        centered_points = points_3d - centroid
        
        # SVD to find the plane normal
        _, _, vh = np.linalg.svd(centered_points)
        normal = vh[-1, :]  # Last row corresponds to smallest singular value
        
        # The plane equation is: normal · (p - centroid) = 0
        # For any 2D point, we can solve for its Z coordinate on the plane
        
        refined_depths = []
        for corner_2d in corners_2d:
            # For point (x, y, z) on the plane, solve for z
            # normal[0] * x + normal[1] * y + normal[2] * z = normal · centroid
            x_cam = (corner_2d[0] - cx) / fx
            y_cam = (corner_2d[1] - cy) / fy
            
            if abs(normal[2]) > 1e-6:  # Avoid division by zero
                z = (np.dot(normal, centroid) - normal[0] * x_cam - normal[1] * y_cam) / normal[2]
                if z > 0.1:  # Sanity check for positive depth
                    refined_depths.append(z)
        
        if refined_depths:
            final_depth = np.mean(refined_depths)
            self.get_logger().info(f"Refined plane depth: {final_depth:.3f}m using plane fitting")
            return final_depth
        else:
            self.get_logger().warn("Plane fitting failed, using initial estimate")
            return initial_depth

    def get_corners_3d_with_depth(self, corners_2d, depth):
        """
        Convert all 2D corners to 3D points using the estimated depth.
        
        Args:
            corners_2d: List of 2D corner points
            depth: Estimated depth of the plane
            
        Returns:
            corners_3d: List of 3D corner points
        """
        corners_3d = []
        for corner_2d in corners_2d:
            corner_3d = self.get_3d_point(corner_2d, depth)
            corners_3d.append(corner_3d)
        return corners_3d

    def get_tf(self, corners, center=None):
        """
        Calculate the transformation matrix from the detected corners.
        
        Args:
            corners: any array-like structure containing the corners of a plane.
        
        Returns:
            A 4x4 transformation matrix representing the pose of the detected plane.
        """
        if center is None:
            center = self.get_center(corners)
        
        # Assuming corners are in the order: lb, rb, rt, lt
        lb, rb, rt, lt = corners
        
        # Calculate the transformation matrix
        tf_matrix = np.eye(4)
        tf_matrix[0:3, 0:3] = np.array([rb - lb, rt - rb, lt - lb]).T
        tf_matrix[0:3, 3] = center
        tf_matrix[0:3, 3] /= np.linalg.norm(tf_matrix[0:3, 3])
        tf_matrix[3, 3] = 1.0
        return tf_matrix

    def visuals(self, img, aptag_info):
        """
        Visualize the detected AprilTags on the image.
        
        Args:
            img: Original image
            aptag_info: Dictionary containing detection results
            
        Returns:
            display_image: Image with visualizations drawn
        """
        if aptag_info is None:
            return img
        
        display_image = img.copy()
        all_ids = aptag_info['ids']
        num_tags = aptag_info['detected_tags']
        all_corners = aptag_info['corners']
        all_centers = aptag_info['centers']
        all_margins = aptag_info['margins']
        plane_center = aptag_info['plane_center']
        estimated_depth = aptag_info.get('estimated_depth', 1.0)
        all_ctr_3d_pts = aptag_info['ctr_3d_points']
        all_corners_3d = aptag_info.get('corners_3d', [])

        # Basic corners and center
        for i, (tag_id, corners, center, margin, ctr_3d) in enumerate(zip(all_ids, all_corners, all_centers, all_margins, all_ctr_3d_pts)):
            print(f"Visualizing Tag {tag_id}: Center {center}, Margin {margin}")

            if corners.shape != (4, 2):
                self.get_logger().error(f"Invalid corners shape for tag {tag_id}: {corners.shape}")
                continue

            # Draw the tag outline (corners)
            corners_int = corners.astype(int)
            cv2.polylines(display_image, [corners_int], True, (0, 255, 0), 2)
            
            # Draw corner points
            for j, corner in enumerate(corners_int):
                cv2.circle(display_image, tuple(corner), 5, (255, 0, 0), -1)
                # Label corners (lb, rb, rt, lt)
                corner_labels = ['LB', 'RB', 'RT', 'LT']
                cv2.putText(display_image, corner_labels[j], 
                            (corner[0] + 10, corner[1] - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            
            # Draw center point
            center_int = center.astype(int)
            cv2.circle(display_image, tuple(center_int), 8, (0, 0, 255), -1)
            
            # Draw tag index near the center
            cv2.putText(display_image, f"Tag {tag_id}", 
                        (center_int[0] - 20, center_int[1] - 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Draw margin and depth info
            cv2.putText(display_image, f"M: {margin:.1f}", 
                        (center_int[0] - 20, center_int[1] + 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Draw 3D center coordinates
            cv2.putText(display_image, f"3D: ({ctr_3d[0]:.2f}, {ctr_3d[1]:.2f}, {ctr_3d[2]:.2f})", 
                        (center_int[0] - 50, center_int[1] + 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
            
            # Visualize the transformation matrix as xyz-axis

        # ------------------------------------------ Basic tags visualization

        # Plane
        if plane_center is not None:
            print(f"Plane center calculated from {num_tags} tags: ({plane_center[0]:.2f}, {plane_center[1]:.2f})")
            plane_center_3d = aptag_info.get('plane_center_3d', None)

            # Draw the plane center as a larger circle with different color
            center_int = plane_center.astype(int)
            cv2.circle(display_image, tuple(center_int), 12, (255, 0, 255), -1)  # Magenta circle
            cv2.circle(display_image, tuple(center_int), 15, (255, 255, 255), 2)   # White border
            
            # Draw label for plane center
            cv2.putText(display_image, "PLANE CENTER", 
                        (center_int[0] - 60, center_int[1] - 25), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
            
            # Draw 2D coordinates
            cv2.putText(display_image, f"2D: ({center_int[0]}, {center_int[1]})", 
                        (center_int[0] - 40, center_int[1] + 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
            
            # Draw 3D coordinates if available
            if plane_center_3d is not None:
                cv2.putText(display_image, f"3D: ({plane_center_3d[0]:.2f}, {plane_center_3d[1]:.2f}, {plane_center_3d[2]:.2f})", 
                            (center_int[0] - 60, center_int[1] + 60), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
        
        # Display estimated depth info in the top-left corner
        cv2.putText(display_image, f"Estimated Depth: {estimated_depth:.3f}m", 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(display_image, f"Tags Detected: {num_tags}", 
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        # ------------------------------------------ Plane center

        return display_image


    def run(self):
        # read camera parameters from a npz file
        try:
            data_dir = os.path.join(get_package_share_directory('pose_gen'), 'cfgs')
            params = np.load(os.path.join(data_dir, 'camera_calibration.npz'))
            camera_matrix = params['camera_matrix']
            dist_coeffs = params['dist_coeffs']
            self.load_camera_params(camera_matrix, dist_coeffs)
        except Exception as e:
            self.get_logger().error(f"Failed to load camera parameters: {e}")
            return
        
        while rclpy.ok():
            self.fps_test_pub.publish(String(data="fps_test_aptag"))

            rclpy.spin_once(self, timeout_sec=0.002) # 500 Hz
            if self.img is None:
                continue

            aptag_info = self.aptag_detection(self.img)
            display_image = self.visuals(self.img, aptag_info)

            # TODO: test tf, get orthogonal matrix, and finally the hover pose that can enter the plane
            if aptag_info is not None:
                self.publish_corners(aptag_info['ctr_3d_points'], aptag_info['plane_center_3d'])
            cv2.imshow("[ Aptag ] AprilTag Detection", display_image)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    tag_size = 0.016 # 16mm tag size
    node = AptagNode(tag_size=tag_size)
    try:
        node.run()
    except Exception as e:
        node.get_logger().error(f"Error occurred: {e}")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()