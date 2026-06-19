import os
import sys
import cv2
import numpy as np
from sklearn.cluster import DBSCAN

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

global shared_color_ranges

class CornerDetector(Node):
    def __init__(self):
        super().__init__('corner_detector')
        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Image, 'corner_image', 10)
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.listener_callback,
            10
        )
        self.get_logger().info('CornerDetector initialized')

    def listener_callback(self, msg):
        image_mat = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

        sharpened_image = self.sharpen_image(image_mat, sigma=10.0)

        global shared_color_ranges
        if 'value' in shared_color_ranges:
            red = shared_color_ranges['value']['red']
            blue = shared_color_ranges['value']['blue']

        hsv_masked_image = self.hsv_mask(sharpened_image, lower_hsv=red[0][0], upper_hsv=red[0][1])
        cv2.imshow("Masked Red", hsv_masked_image)

        corners = self.detect_corners(hsv_masked_image)
        # corners = self.clustering_corners(corners=corners, eps=25, min_samples=2)
        annotated_image = self.annotate_corners(image_mat, corners)
        cv2.imshow("Camera recv", annotated_image)
        cv2.waitKey(1)
        msg = self.bridge.cv2_to_imgmsg(image_mat, encoding="rgb8")
        self.publisher_.publish(msg)

    def detect_corners(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners = cv2.goodFeaturesToTrack(gray, maxCorners=1000, qualityLevel=0.01, minDistance=10)
        return corners

    def annotate_corners(self, img, corners):
        if corners is not None:
            corners = corners.astype(np.intp)
            for i in corners:
                x, y = i.ravel()
                cv2.circle(img, (x, y), 3, (0, 255, 0), -1)
        return img
    
    def sharpen_image(self, img, sigma=1.0):
        """
        Apply Gaussian blur and subtract from original to sharpen the image.
        :param img: Input image.
        :param sigma: Standard deviation for Gaussian kernel.
        :return: Sharpened image.
        """
        blurred = cv2.GaussianBlur(img, (0, 0), sigma)
        sharpened = cv2.addWeighted(img, 1.5, blurred, -0.5, 0)
        return sharpened

    def hsv_mask(self, img, lower_hsv, upper_hsv):
        """
        Apply HSV mask to the image.
        :param img: Input image.
        :param lower_hsv: Lower HSV threshold.
        :param upper_hsv: Upper HSV threshold.
        :return: Masked image.
        """
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
        masked_img = cv2.bitwise_and(img, img, mask=mask)
        return masked_img


import threading
import tkinter as tk
from tkinter import ttk
def run_hsv_gui():
    global shared_color_ranges
    get_hsv_thresholds_gui(shared_color_ranges)

def get_hsv_thresholds_gui(shared_color_ranges):
    """
    Launch a Tkinter GUI to control HSV thresholds for red (lower/upper) and blue.
    Updates the shared_color_ranges dict in real time as sliders move.
    """
    def update_color_ranges(*args):
        shared_color_ranges['value'] = {
            'red': [
                (tuple(sliders['red_min', i].get() for i in range(3)), tuple(sliders['red_max', i].get() for i in range(3))),
            ],
            'blue': [
                (tuple(sliders['blue_min', i].get() for i in range(3)), tuple(sliders['blue_max', i].get() for i in range(3)))
            ]
        }

    root = tk.Tk()
    root.title("HSV Threshold Controller")

    hsv_vars = {
        'red_min': [0, 150, 120], 'red_max': [10, 255, 255],
        'blue_min': [100, 150, 0], 'blue_max': [140, 255, 255]
    }
    sliders = {}

    def make_dual_sliders(frame, label_prefix, min_key, max_key):
        for i, name in enumerate(['H', 'S', 'V']):
            lmin = tk.Label(frame, text=f"{label_prefix} {name} Min")
            lmin.grid(row=i, column=0)
            vmin = tk.IntVar(value=hsv_vars[min_key][i])
            smin = tk.Scale(frame, from_=0, to=180 if name == 'H' else 255, orient=tk.HORIZONTAL, variable=vmin)
            smin.grid(row=i, column=1)
            sliders[min_key, i] = vmin
            vmin.trace_add('write', update_color_ranges)

            lmax = tk.Label(frame, text=f"{label_prefix} {name} Max")
            lmax.grid(row=i, column=2)
            vmax = tk.IntVar(value=hsv_vars[max_key][i])
            smax = tk.Scale(frame, from_=0, to=180 if name == 'H' else 255, orient=tk.HORIZONTAL, variable=vmax)
            smax.grid(row=i, column=3)
            sliders[max_key, i] = vmax
            vmax.trace_add('write', update_color_ranges)

    notebook = ttk.Notebook(root)
    notebook.pack(fill='both', expand=True)

    # Red lower/upper 
    fred1 = tk.Frame(notebook)
    make_dual_sliders(fred1, "Red", 'red_min', 'red_max')
    notebook.add(fred1, text="Red Lower")

    # Blue lower/upper
    fblue = tk.Frame(notebook)
    make_dual_sliders(fblue, "Blue", 'blue_min', 'blue_max')
    notebook.add(fblue, text="Blue")

    update_color_ranges()  # Set initial values

    root.mainloop()


def main(args=None):

    global shared_color_ranges

    shared_color_ranges = {
        'red': [
            ((0, 100, 100), (10, 255, 255)),    # lower red, stricter S and V
            ((170, 100, 100), (180, 255, 255))  # upper red, stricter S and V
        ],
        'blue': [
            ((100, 100, 0), (140, 255, 255))    # blue range
        ]
    }
    gui_thread = threading.Thread(target=run_hsv_gui, daemon=True)
    gui_thread.start()

    rclpy.init(args=args)
    corner_detector = CornerDetector()
    rclpy.spin(corner_detector)
    corner_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
