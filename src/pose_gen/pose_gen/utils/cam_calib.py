import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import os
import time

class CameraCalibrationNode(Node):
    def __init__(self):
        super().__init__('camera_calibration_node')
        self.cam_sub = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.img = None
        self.bridge = CvBridge()
        
        # Calibration parameters
        self.chessboard_size = (8, 11)  # Inner corners (width, height)
        self.square_size = 0.03  # 30mm squares

        # Storage for calibration data
        self.objpoints = []  # 3D points in real world space
        self.imgpoints = []  # 2D points in image plane
        self.collected_images = []
        self.calibration_images_needed = 20
        
        # Calibration results
        self.camera_matrix = None
        self.dist_coeffs = None
        self.rvecs = None
        self.tvecs = None
        
        # Visualization parameters
        self.show_visualization = True
        self.last_detection_time = 0
        self.detection_cooldown = 2.0  # seconds between detections
        
        # Prepare object points
        self.objp = np.zeros((self.chessboard_size[0] * self.chessboard_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.chessboard_size[0], 0:self.chessboard_size[1]].T.reshape(-1, 2)
        self.objp *= self.square_size
        
        self.get_logger().info("Camera Calibration Node initialized.")
        self.get_logger().info("Press 'q' in the visualization window to quit.")

    def image_callback(self, msg):
        """
        Callback function to handle incoming camera images.
        This function can be used to capture images for calibration.
        """
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            
            # Show live visualization
            if self.show_visualization:
                self.visualize_camera_feed()
                
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            self.img = None
    
    def visualize_camera_feed(self):
        """
        Display the camera feed with chessboard detection visualization.
        """
        if self.img is None:
            return
            
        display_img = self.img.copy()
        gray = cv2.cvtColor(self.img, cv2.COLOR_RGB2GRAY)

        # Try to find chessboard corners for visualization
        ret, corners = cv2.findChessboardCorners(gray, self.chessboard_size, None)
        
        # Draw status information
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.7
        thickness = 2
        
        # Status text
        status_y = 30
        cv2.putText(display_img, f"Images collected: {len(self.collected_images)}/{self.calibration_images_needed}", 
                   (10, status_y), font, font_scale, (255, 255, 255), thickness)
        
        status_y += 30
        if len(self.collected_images) >= self.calibration_images_needed:
            cv2.putText(display_img, "Ready for calibration!", 
                       (10, status_y), font, font_scale, (0, 255, 0), thickness)
        else:
            cv2.putText(display_img, "Move chessboard to different positions", 
                       (10, status_y), font, font_scale, (0, 255, 255), thickness)
        
        # Draw chessboard detection
        if ret:
            # Draw the corners
            cv2.drawChessboardCorners(display_img, self.chessboard_size, corners, ret)
            
            # Add detection indicator
            cv2.putText(display_img, "CHESSBOARD DETECTED!", 
                       (10, display_img.shape[0] - 20), font, font_scale, (0, 255, 0), thickness)
            
            # Draw detection rectangle around the pattern
            x_coords = corners[:, 0, 0]
            y_coords = corners[:, 0, 1]
            x_min, x_max = int(np.min(x_coords)) - 20, int(np.max(x_coords)) + 20
            y_min, y_max = int(np.min(y_coords)) - 20, int(np.max(y_coords)) + 20
            cv2.rectangle(display_img, (x_min, y_min), (x_max, y_max), (0, 255, 0), 3)
            
        else:
            # No chessboard detected
            cv2.putText(display_img, "No chessboard detected", 
                       (10, display_img.shape[0] - 20), font, font_scale, (0, 0, 255), thickness)
        
        # Add instructions
        instructions_y = display_img.shape[0] - 80
        cv2.putText(display_img, "Press 'q' to quit, 'c' to capture when detected", 
                   (10, instructions_y), font, 0.5, (255, 255, 255), 1)
        
        # Display the image
        cv2.imshow('Camera Calibration - Live Feed', display_img)
        
        # Handle key presses
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info("Quitting calibration...")
            cv2.destroyAllWindows()
            rclpy.shutdown()
        elif key == ord('c') and ret:
            # Manual capture when 'c' is pressed and chessboard is detected
            current_time = time.time()
            if current_time - self.last_detection_time > self.detection_cooldown:
                self.capture_calibration_image(corners)
                self.last_detection_time = current_time
    
    def capture_calibration_image(self, corners):
        """
        Capture a calibration image with detected corners.
        """
        if self.img is None:
            return False
            
        gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        
        # Refine corner positions
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        
        # Store the corners
        self.objpoints.append(self.objp)
        self.imgpoints.append(corners2)
        self.collected_images.append(self.img.copy())
        
        self.get_logger().info(f"Image captured! Total images: {len(self.collected_images)}")
        return True

    def chessboard_detection(self):
        """
        Detect chessboard corners in the current image.
        Returns True if corners are found and added to calibration data.
        """
        if self.img is None:
            return False
            
        # Check cooldown to prevent rapid successive captures
        current_time = time.time()
        if current_time - self.last_detection_time < self.detection_cooldown:
            return False
            
        gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        
        # Find chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, self.chessboard_size, None)
        
        if ret:
            success = self.capture_calibration_image(corners)
            if success:
                self.last_detection_time = current_time
                remaining = self.calibration_images_needed - len(self.collected_images)
                if remaining > 0:
                    self.get_logger().info(f"Images remaining: {remaining}")
                return True
        
        return False

    def img_collection(self):
        """
        Collect calibration images by detecting chessboard patterns.
        """
        if len(self.collected_images) >= self.calibration_images_needed:
            self.get_logger().info(f"Sufficient images collected ({len(self.collected_images)}). Ready for calibration.")
            return True
            
        # Try to detect chessboard in current image
        if self.chessboard_detection():
            remaining = self.calibration_images_needed - len(self.collected_images)
            self.get_logger().info(f"Images needed: {remaining}")
            
        return len(self.collected_images) >= self.calibration_images_needed

    def calibration(self):
        """
        Perform camera calibration using collected images.
        """
        if len(self.objpoints) < 10:
            self.get_logger().error("Not enough calibration images. Need at least 10 images.")
            return False
            
        self.get_logger().info("Starting camera calibration...")
        
        # Get image size
        img_size = (self.collected_images[0].shape[1], self.collected_images[0].shape[0])
        
        # Perform calibration
        ret, self.camera_matrix, self.dist_coeffs, self.rvecs, self.tvecs = cv2.calibrateCamera(
            self.objpoints, self.imgpoints, img_size, None, None
        )
        
        if ret:
            self.get_logger().info("Camera calibration completed successfully!")
            self.get_logger().info(f"RMS reprojection error: {ret:.4f}")
            return True
        else:
            self.get_logger().error("Camera calibration failed!")
            return False

    def validation(self):
        """
        Validate calibration by calculating reprojection error and undistorting test images.
        """
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.get_logger().error("No calibration data available for validation.")
            return False
            
        # Calculate mean reprojection error
        total_error = 0
        for i in range(len(self.objpoints)):
            imgpoints2, _ = cv2.projectPoints(
                self.objpoints[i], self.rvecs[i], self.tvecs[i], 
                self.camera_matrix, self.dist_coeffs
            )
            error = cv2.norm(self.imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
            total_error += error
            
        mean_error = total_error / len(self.objpoints)
        self.get_logger().info(f"Mean reprojection error: {mean_error:.4f} pixels")
        
        # Validate with undistortion test
        if len(self.collected_images) > 0:
            test_img = self.collected_images[0]
            undistorted = cv2.undistort(test_img, self.camera_matrix, self.dist_coeffs)
            self.get_logger().info("Undistortion test completed successfully.")
            
        return mean_error < 1.0  # Good calibration should have error < 0.5-1.0 pixels

    def save_calibration(self, save_dir='src/RM_Engineer/pose_gen/cfgs'):
        """
        Save calibration parameters to file.
        """
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.get_logger().error("No calibration data to save.")
            return False
            
        try:
            # Create the save directory if it doesn't exist
            os.makedirs(save_dir, exist_ok=True)
            
            # Create calibration data dictionary
            calibration_data = {
                'camera_matrix': self.camera_matrix.tolist(),
                'distortion_coefficients': self.dist_coeffs.tolist(),
                'image_size': [self.collected_images[0].shape[1], self.collected_images[0].shape[0]],
                'calibration_date': time.strftime("%Y-%m-%d %H:%M:%S"),
                'num_images_used': len(self.collected_images)
            }

            # Save to numpy file
            save_path = os.path.join(save_dir, 'camera_calibration.npz')
            np.savez(save_path,
                    camera_matrix=self.camera_matrix,
                    dist_coeffs=self.dist_coeffs)

            # Also save as readable text
            save_path = os.path.join(save_dir, 'camera_calibration.txt')
            with open(save_path, 'w') as f:
                f.write("Camera Calibration Results\n")
                f.write("==========================\n\n")
                f.write(f"Camera Matrix:\n{self.camera_matrix}\n\n")
                f.write(f"Distortion Coefficients:\n{self.dist_coeffs}\n\n")
                f.write(f"Number of images used: {len(self.collected_images)}\n")
                
            self.get_logger().info("Calibration data saved to camera_calibration.npz and camera_calibration.txt")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to save calibration data: {e}")
            return False

    def run(self):
        """
        Main loop for the camera calibration process.
        This function implements the complete calibration workflow.
        """
        self.get_logger().info("Running camera calibration...")
        self.get_logger().info("Move the chessboard in front of the camera to different positions and orientations.")
        self.get_logger().info("Press 'c' to manually capture when chessboard is detected, or wait for automatic capture.")
        
        calibration_completed = False
        
        try:
            while rclpy.ok() and not calibration_completed:
                rclpy.spin_once(self, timeout_sec=0.1)
                
                if self.img is not None:
                    # Phase 1: Collect calibration images
                    if not self.img_collection():
                        continue
                        
                    # Disable visualization during calibration processing
                    self.show_visualization = False
                    cv2.destroyAllWindows()
                    
                    # Phase 2: Perform calibration
                    self.get_logger().info("Starting calibration process...")
                    if not self.calibration():
                        self.get_logger().error("Calibration failed. Exiting.")
                        break
                        
                    # Phase 3: Validate calibration
                    if self.validation():
                        self.get_logger().info("Calibration validation passed!")
                    else:
                        self.get_logger().warning("Calibration validation failed, but proceeding...")
                        
                    # Phase 4: Save calibration results
                    if self.save_calibration():
                        self.get_logger().info("Calibration process completed successfully!")
                        calibration_completed = True
                    else:
                        self.get_logger().error("Failed to save calibration data.")
                        break
                        
        except KeyboardInterrupt:
            self.get_logger().info("Calibration interrupted by user.")
        finally:
            # Cleanup
            cv2.destroyAllWindows()
            self.get_logger().info("Calibration node shutting down.")

def main():
    rclpy.init()
    node = CameraCalibrationNode()
    node.run()
    rclpy.shutdown()

if __name__ == "__main__":
    main()