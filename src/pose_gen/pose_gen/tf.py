import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

import numpy as np
import json
import time


class Transform(Node):
    def __init__(self):
        super().__init__('transform_node')
        self.fps_test_pub = self.create_publisher(String, '/fps_test_transform', 10)
        self.pose_pub = self.create_publisher(TransformStamped, '/hover_pose', 10)
        self.viz_pub = self.create_publisher(MarkerArray, '/debug_visualization', 10)
        self.corner_sub = self.create_subscription(String, '/corner_aptag', self.corner_callback, 10)
        
        self.corners = None
        
        # Extended Kalman Filter initialization for 4 corners
        # State vector: [x1,y1,z1,vx1,vy1,vz1, x2,y2,z2,vx2,vy2,vz2, ...] for 4 corners
        self.state_dim = 24  # 4 corners * 6 states each (position + velocity)
        self.measurement_dim = 12  # 4 corners * 3 coordinates each
        
        # State vector: [x, y, z, vx, vy, vz] for each of 4 corners
        self.x = np.zeros(self.state_dim)  # State estimate
        self.P = np.eye(self.state_dim) * 100.0  # Covariance matrix (high initial uncertainty)
        
        # Process noise covariance
        self.Q = np.eye(self.state_dim) * 0.1
        
        # Measurement noise covariance
        self.R = np.eye(self.measurement_dim) * 1.0
        
        # Time tracking
        self.last_time = None
        self.filter_initialized = False
        # ------------------------------------- EKF
        
        # Hover pose configuration
        self.hover_distance = 0.5  # Distance above the plane (in meters)
        self.hover_height_offset = 0.2  # Additional height offset for safety
        # ------------------------------------- hover pose configuration

    def corner_callback(self, msg):
        # corners received always in order of [lb, rb, rt, lt]
        self.corners = json.loads(msg.data)
        self.get_logger().info(f"Received corners: {self.corners}")

    def get_corners(self):
        """
        Get the corners from the subscription.
        """
        if self.corners is None:
            self.get_logger().warn("No corners received yet.")
            return None
        return np.array(self.corners)

    def karman_filter_e(self, corners):
        """
        Apply Extended Kalman filter to the corners.
        Input: corners - numpy array of shape (4, 3) containing [[x,y,z], [x,y,z], [x,y,z], [x,y,z]]
        Output: filtered corners in the same format
        """
        if corners is None:
            return None
        
        # Ensure corners has the right shape
        if corners.shape != (4, 3):
            self.get_logger().warn(f"Invalid corner shape: {corners.shape}, expected (4, 3)")
            return corners
        
        current_time = time.time()
        
        # Initialize filter on first measurement
        if not self.filter_initialized:
            # Initialize state with first measurement (positions) and zero velocities
            for i in range(4):
                self.x[i*6:(i*6+3)] = corners[i]  # positions
                self.x[i*6+3:(i*6+6)] = 0.0       # velocities
            self.last_time = current_time
            self.filter_initialized = True
            return corners
        
        # Calculate time step
        dt = current_time - self.last_time
        self.last_time = current_time
        
        if dt <= 0 or dt > 1.0:  # Skip if time step is invalid
            return corners
        
        # === PREDICTION STEP ===
        # State transition matrix F (constant velocity model)
        F = np.eye(self.state_dim)
        for i in range(4):
            # For each corner: position = position + velocity * dt
            base_idx = i * 6
            F[base_idx, base_idx + 3] = dt      # x += vx * dt
            F[base_idx + 1, base_idx + 4] = dt  # y += vy * dt
            F[base_idx + 2, base_idx + 5] = dt  # z += vz * dt
        
        # Prediction
        self.x = F @ self.x  # State prediction
        self.P = F @ self.P @ F.T + self.Q  # Covariance prediction
        
        # === UPDATE STEP ===
        # Measurement matrix H (we observe positions only)
        H = np.zeros((self.measurement_dim, self.state_dim))
        for i in range(4):
            base_meas_idx = i * 3
            base_state_idx = i * 6
            H[base_meas_idx, base_state_idx] = 1.0        # observe x
            H[base_meas_idx + 1, base_state_idx + 1] = 1.0  # observe y
            H[base_meas_idx + 2, base_state_idx + 2] = 1.0  # observe z
        
        # Measurement vector
        z = corners.flatten()  # Convert (4,3) to (12,) vector
        
        # Innovation
        y = z - H @ self.x  # Innovation (measurement residual)
        S = H @ self.P @ H.T + self.R  # Innovation covariance
        
        # Kalman gain
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            self.get_logger().warn("Kalman filter: Singular matrix in gain calculation")
            return corners
        
        # Update state and covariance
        self.x = self.x + K @ y
        I_KH = np.eye(self.state_dim) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T  # Joseph form for numerical stability
        
        # Extract filtered positions and reshape back to (4, 3)
        filtered_corners = np.zeros((4, 3))
        for i in range(4):
            filtered_corners[i] = self.x[i*6:(i*6+3)]
        
        return filtered_corners

    def get_plane(self, corners):
        """
        Calculate the plane from the 4 corners.
        Input: corners - numpy array of shape (4, 3) containing [[x,y,z], [x,y,z], [x,y,z], [x,y,z]]
        Output: dictionary containing:
                - 'center': plane center point [x, y, z]
                - 'normal': unit normal vector pointing away from plane [x, y, z]
                - 'equation': plane equation coefficients [a, b, c, d] where ax + by + cz + d = 0
        """
        if corners is None or len(corners) < 3:
            self.get_logger().warn("Not enough corners to calculate a plane.")
            return None
        
        if corners.shape != (4, 3):
            self.get_logger().warn(f"Invalid corner shape: {corners.shape}, expected (4, 3)")
            return None
        
        try:
            plane_center = np.mean(corners, axis=0)
            
            # For 4 corners that are all coplanar (z=0), use cross product method first
            # This is more reliable for planar AprilTag corners
            diagonal1 = corners[2] - corners[0]  # Corner 0 to corner 2
            diagonal2 = corners[3] - corners[1]  # Corner 1 to corner 3
            
            # Cross product gives normal vector
            cross_normal = np.cross(diagonal1, diagonal2)
            cross_normal_magnitude = np.linalg.norm(cross_normal)
            
            if cross_normal_magnitude > 1e-6:  # Check if vectors are not parallel
                normal = cross_normal / cross_normal_magnitude
                if normal[2] < 0:
                    normal = -normal
                a, b, c = normal
                d = -np.dot(normal, plane_center)
            else:
                # Fallback: Use SVD method if cross product fails
                centered_corners = corners - plane_center
                
                # Perform SVD on centered points (4x3 matrix)
                U, S, Vt = np.linalg.svd(centered_corners)
                
                # Check dimensions and select appropriate normal vector
                if U.shape[1] >= 3:
                    normal = U[:, -1]  # Last column of U
                else:
                    # If U doesn't have enough columns, use alternative approach
                    normal = np.array([0, 0, 1])  # Default to Z-up
                
                # Ensure we have exactly 3 components
                if len(normal) != 3:
                    normal = np.array([0, 0, 1])
                
                # Ensure consistent direction
                if normal[2] < 0:
                    normal = -normal
                normal = normal / np.linalg.norm(normal)
                a, b, c = normal
                d = -np.dot(normal, plane_center)

            # Verify plane quality by checking how well points fit
            max_distance = 0
            for corner in corners:
                distance = abs(np.dot(normal, corner) + d)
                max_distance = max(max_distance, distance)
            
            if max_distance > 0.1:  # Threshold for plane fitting quality
                self.get_logger().warn(f"Poor plane fit, max distance: {max_distance:.4f}")

            plane_info = {
                'center': plane_center,
                'normal': normal,
                'equation': np.array([a, b, c, d]),
                'fit_quality': max_distance
            }
            
            return plane_info
            
        except np.linalg.LinAlgError as e:
            self.get_logger().error(f"Linear algebra error in plane calculation: {e}")
            return None
        except Exception as e:
            self.get_logger().error(f"Unexpected error in plane calculation: {e}")
            return None

    def get_hover_pose(self, plane_info):
        """
        Calculate the hover pose based on the plane information.
        The hover pose is positioned at a certain distance away from the plane
        and oriented to point toward the plane center.
        
        Input: plane_info - dictionary containing 'center' and 'normal'
        Output: hover pose dictionary with position and orientation (quaternion)
        """
        if plane_info is None:
            self.get_logger().warn("No plane information provided.")
            return None
        
        try:
            plane_center = plane_info['center']
            plane_normal = plane_info['normal']
            
            # Calculate hover position: plane center + distance along normal vector
            hover_position = plane_center + (self.hover_distance + self.hover_height_offset) * plane_normal
            
            # Calculate orientation quaternion to point toward plane center
            # The hover pose should have its Z-axis (forward direction) pointing toward the plane center
            
            # Direction vector from hover position to plane center (this will be our -Z axis)
            look_direction = hover_position - plane_center
            look_direction = look_direction / np.linalg.norm(look_direction)
            
            # Choose an arbitrary up direction (world Z-axis) and create orthogonal frame
            z_axis = look_direction
            world_up = np.array([0, 0, 1])
            if abs(np.dot(z_axis, world_up)) > 0.95:
                world_up = np.array([0, 1, 0])
            x_axis = np.cross(world_up, z_axis)
            x_axis = x_axis / np.linalg.norm(x_axis)  # Normalize
            y_axis = np.cross(z_axis, x_axis)
            y_axis = y_axis / np.linalg.norm(y_axis)  # Normalize
            rotation_matrix = np.column_stack([x_axis, y_axis, z_axis])
            
            # Convert rotation matrix to quaternion
            quaternion = self._rotation_matrix_to_quaternion(rotation_matrix)
            
            hover_pose = {
                'position': hover_position.tolist(),
                'orientation': quaternion.tolist()  # [x, y, z, w]
            }
            
            # Debug information
            distance_to_center = np.linalg.norm(hover_position - plane_center)
            self.get_logger().debug(f"Hover pose calculated: distance={distance_to_center:.3f}m")
            
            return hover_pose
            
        except KeyError as e:
            self.get_logger().error(f"Missing key in plane_info: {e}")
            return None
        except Exception as e:
            self.get_logger().error(f"Error calculating hover pose: {e}")
            return None
    
    def _rotation_matrix_to_quaternion(self, rotation_matrix):
        """
        Convert a 3x3 rotation matrix to a quaternion [x, y, z, w].
        Uses Shepperd's method for numerical stability.
        """
        R = rotation_matrix
        
        # Shepperd's method
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        
        if trace > 0:
            s = np.sqrt(trace + 1.0) * 2  # s = 4 * qw
            qw = 0.25 * s
            qx = (R[2, 1] - R[1, 2]) / s
            qy = (R[0, 2] - R[2, 0]) / s
            qz = (R[1, 0] - R[0, 1]) / s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2  # s = 4 * qx
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2  # s = 4 * qy
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2  # s = 4 * qz
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s
        
        # Return quaternion in [x, y, z, w] format
        quaternion = np.array([qx, qy, qz, qw])
        
        # Normalize quaternion
        quaternion = quaternion / np.linalg.norm(quaternion)
        
        return quaternion

    def publish_hover_pose(self, hover_pose):
        """
        Publish the hover pose as a TransformStamped message.
        
        Args:
            hover_pose: Dictionary containing position and orientation.
        """
        transform_msg = TransformStamped()
        transform_msg.header.stamp = self.get_clock().now().to_msg()
        transform_msg.header.frame_id = 'map'
        transform_msg.child_frame_id = 'hover_pose'

        transform_msg.transform.translation.x = hover_pose['position'][0]
        transform_msg.transform.translation.y = hover_pose['position'][1]
        transform_msg.transform.translation.z = hover_pose['position'][2]

        transform_msg.transform.rotation.x = hover_pose['orientation'][0]
        transform_msg.transform.rotation.y = hover_pose['orientation'][1]
        transform_msg.transform.rotation.z = hover_pose['orientation'][2]
        transform_msg.transform.rotation.w = hover_pose['orientation'][3]

        self.pose_pub.publish(transform_msg)
        self.get_logger().info(f"Published hover pose: {hover_pose}")

    def visualize_debug(self, corners, plane_info, hover_pose=None):
        """
        Visualize debug information for corners and plane.
        Show in RViz2 the corners, the calculated plane, the center, and the normal vector, and the hover pose.
        """
        if corners is None or plane_info is None:
            return
        
        marker_array = MarkerArray()
        current_time = self.get_clock().now().to_msg()
        
        # Clear previous markers
        clear_marker = Marker()
        clear_marker.header.frame_id = 'map'
        clear_marker.header.stamp = current_time
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        
        # 1. Visualize corners as spheres
        for i, corner in enumerate(corners):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = current_time
            marker.ns = 'corners'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Position
            marker.pose.position.x = float(corner[0])
            marker.pose.position.y = float(corner[1])
            marker.pose.position.z = float(corner[2])
            marker.pose.orientation.w = 1.0
            
            # Scale
            marker.scale.x = 0.02
            marker.scale.y = 0.02
            marker.scale.z = 0.02
            
            # Color (different colors for each corner)
            colors = [
                [1.0, 0.0, 0.0],  # Red
                [0.0, 1.0, 0.0],  # Green
                [0.0, 0.0, 1.0],  # Blue
                [1.0, 1.0, 0.0]   # Yellow
            ]
            marker.color.r = colors[i][0]
            marker.color.g = colors[i][1]
            marker.color.b = colors[i][2]
            marker.color.a = 1.0
            
            marker_array.markers.append(marker)
        
        # 2. Visualize plane center as a larger sphere
        center_marker = Marker()
        center_marker.header.frame_id = 'map'
        center_marker.header.stamp = current_time
        center_marker.ns = 'plane_center'
        center_marker.id = 0
        center_marker.type = Marker.SPHERE
        center_marker.action = Marker.ADD
        
        plane_center = plane_info['center']
        center_marker.pose.position.x = float(plane_center[0])
        center_marker.pose.position.y = float(plane_center[1])
        center_marker.pose.position.z = float(plane_center[2])
        center_marker.pose.orientation.w = 1.0
        
        center_marker.scale.x = 0.02
        center_marker.scale.y = 0.02
        center_marker.scale.z = 0.02
        
        center_marker.color.r = 1.0
        center_marker.color.g = 0.5
        center_marker.color.b = 0.0  # Orange
        center_marker.color.a = 1.0
        
        marker_array.markers.append(center_marker)
        
        # 3. Visualize plane normal as an arrow
        normal_marker = Marker()
        normal_marker.header.frame_id = 'map'
        normal_marker.header.stamp = current_time
        normal_marker.ns = 'plane_normal'
        normal_marker.id = 0
        normal_marker.type = Marker.ARROW
        normal_marker.action = Marker.ADD
        
        # Arrow from plane center along normal direction
        plane_normal = plane_info['normal']
        arrow_length = 0.3
        
        # Start point
        start_point = Point()
        start_point.x = float(plane_center[0])
        start_point.y = float(plane_center[1])
        start_point.z = float(plane_center[2])
        
        # End point
        end_point = Point()
        end_point.x = float(plane_center[0] + plane_normal[0] * arrow_length)
        end_point.y = float(plane_center[1] + plane_normal[1] * arrow_length)
        end_point.z = float(plane_center[2] + plane_normal[2] * arrow_length)
        
        normal_marker.points = [start_point, end_point]
        
        normal_marker.scale.x = 0.02  # Arrow shaft diameter
        normal_marker.scale.y = 0.04  # Arrow head diameter
        normal_marker.scale.z = 0.06  # Arrow head length
        
        normal_marker.color.r = 0.0
        normal_marker.color.g = 1.0
        normal_marker.color.b = 1.0  # Cyan
        normal_marker.color.a = 1.0
        
        marker_array.markers.append(normal_marker)
        
        # 4. Visualize plane as a mesh (rectangle connecting corners)
        plane_marker = Marker()
        plane_marker.header.frame_id = 'map'
        plane_marker.header.stamp = current_time
        plane_marker.ns = 'plane_surface'
        plane_marker.id = 0
        plane_marker.type = Marker.TRIANGLE_LIST
        plane_marker.action = Marker.ADD
        
        # Create triangles to represent the plane (two triangles make a rectangle)
        # Triangle 1: corners[0], corners[1], corners[2]
        for i in [0, 1, 2]:
            point = Point()
            point.x = float(corners[i][0])
            point.y = float(corners[i][1])
            point.z = float(corners[i][2])
            plane_marker.points.append(point)
        
        # Triangle 2: corners[0], corners[2], corners[3]
        for i in [0, 2, 3]:
            point = Point()
            point.x = float(corners[i][0])
            point.y = float(corners[i][1])
            point.z = float(corners[i][2])
            plane_marker.points.append(point)
        
        plane_marker.scale.x = 1.0
        plane_marker.scale.y = 1.0
        plane_marker.scale.z = 1.0
        
        plane_marker.color.r = 0.5
        plane_marker.color.g = 0.5
        plane_marker.color.b = 1.0  # Light blue
        plane_marker.color.a = 0.3   # Semi-transparent
        
        marker_array.markers.append(plane_marker)
        
        # 5. Visualize hover pose if provided
        if hover_pose is not None:
            hover_marker = Marker()
            hover_marker.header.frame_id = 'map'
            hover_marker.header.stamp = current_time
            hover_marker.ns = 'hover_pose'
            hover_marker.id = 0
            hover_marker.type = Marker.CUBE
            hover_marker.action = Marker.ADD
            
            hover_position = hover_pose['position']
            hover_orientation = hover_pose['orientation']
            
            hover_marker.pose.position.x = float(hover_position[0])
            hover_marker.pose.position.y = float(hover_position[1])
            hover_marker.pose.position.z = float(hover_position[2])
            
            hover_marker.pose.orientation.x = float(hover_orientation[0])
            hover_marker.pose.orientation.y = float(hover_orientation[1])
            hover_marker.pose.orientation.z = float(hover_orientation[2])
            hover_marker.pose.orientation.w = float(hover_orientation[3])
            
            hover_marker.scale.x = 0.2
            hover_marker.scale.y = 0.2
            hover_marker.scale.z = 0.1
            
            hover_marker.color.r = 1.0
            hover_marker.color.g = 0.0
            hover_marker.color.b = 1.0  # Magenta
            hover_marker.color.a = 0.8
            
            marker_array.markers.append(hover_marker)
            
            # Add direction arrow from hover pose to plane center
            direction_marker = Marker()
            direction_marker.header.frame_id = 'map'
            direction_marker.header.stamp = current_time
            direction_marker.ns = 'hover_direction'
            direction_marker.id = 0
            direction_marker.type = Marker.ARROW
            direction_marker.action = Marker.ADD
            
            # Arrow from hover pose to plane center
            hover_start = Point()
            hover_start.x = float(hover_position[0])
            hover_start.y = float(hover_position[1])
            hover_start.z = float(hover_position[2])
            
            hover_end = Point()
            hover_end.x = float(plane_center[0])
            hover_end.y = float(plane_center[1])
            hover_end.z = float(plane_center[2])
            
            direction_marker.points = [hover_start, hover_end]
            
            direction_marker.scale.x = 0.015  # Arrow shaft diameter
            direction_marker.scale.y = 0.03   # Arrow head diameter
            direction_marker.scale.z = 0.05   # Arrow head length
            
            direction_marker.color.r = 1.0
            direction_marker.color.g = 0.0
            direction_marker.color.b = 1.0  # Magenta
            direction_marker.color.a = 0.6
            
            marker_array.markers.append(direction_marker)
        
        # Publish all markers
        self.viz_pub.publish(marker_array)


    def run(self):
        while rclpy.ok():
            self.fps_test_pub.publish(String(data="fps_test_transform"))

            # Get corners            
            rclpy.spin_once(self, timeout_sec=0.002)
            corners = self.get_corners()
            # karman God
            corners = self.karman_filter_e(corners)
            # Get the plane
            plane_info = self.get_plane(corners)
            # Get the hover pose
            hover_pose = self.get_hover_pose(plane_info)
            if hover_pose is not None:
                self.publish_hover_pose(hover_pose)
                # Visualize debug information
                self.visualize_debug(corners, plane_info, hover_pose)
        self.get_logger().info("Transform node is shutting down.")

def main(args=None):
    rclpy.init(args=args)
    tf_node = Transform()
    try:
        tf_node.run()
    except Exception as e:
        tf_node.get_logger().error(f"Error occurred: {e}")
    tf_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()