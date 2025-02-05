#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge
import numpy as np
import os
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp"

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.publisher_ids = self.create_publisher(Int32MultiArray, 'detected_ids', 10)
        self.publisher_frame = self.create_publisher(Image, 'detected_frame', 10)
        
        self.bridge = CvBridge()
        
        # Load image from parameter
        self.declare_parameter('image_path', '../marker_1.png') 
        self.image_path = self.get_parameter('image_path').value
        
        # ArUco predefined dictionary and parameters
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        
        # Capture video frames using rtsp
        self.cap = cv2.VideoCapture("rtsp://admin:123456@192.168.1.14:554/mpeg4", cv2.CAP_FFMPEG) # Replace with camera RTSP address
        
        # self.detect_aruco()
        
        # Pose estimation (dummy camera params for now)
        self.camera_matrix = np.array([[800, 0, 320], [0, 800, 240], [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.array([0, 0, 0, 0], dtype=np.float32)
        self.marker_length = 0.05
            
        self.get_logger().info("Aruco detection node has been started.")
        self.timer = self.create_timer(0.1, self.detect_aruco)  # Timer to run at 10 Hz
        
    def detect_aruco(self):
        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().warning("Failed to capture frame from camera.")
            frame = cv2.imread(self.image_path)
            if frame is None:
                self.get_logger().error(f"Failed to load image from {self.image_path}")
                return
        
        # Add a white border (quiet zone)
        quiet_zone_size = 20
        image = cv2.copyMakeBorder(
            frame,
            quiet_zone_size,
            quiet_zone_size,
            quiet_zone_size,
            quiet_zone_size,
            cv2.BORDER_CONSTANT,
            value=(255, 0, 0),
        )

        # Detect markers
        corners, ids, _ = self.detector.detectMarkers(image)
        
        if ids is not None:
            self.get_logger().info(f"Detected ArUco IDs: {ids.flatten()}")
            self.get_logger().info(f"Detected ArUco corners: {corners}")
            
            # Publish the detected IDs
            msg_ids = Int32MultiArray(data=ids.flatten().tolist())
            self.publisher_ids.publish(msg_ids)

            # Draw detected markers
            for i in range(len(ids)):
                # Draw borders around detected markers
                cv2.polylines(frame, [np.int32(corners[i])], True, (255, 0, 0), 2)
                cv2.putText(image, f"ID: {ids[i][0]}", (int(corners[i][0][0][0]), int(corners[i][0][0][1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], self.marker_length, self.camera_matrix, self.dist_coeffs)
                cv2.drawFrameAxes(image, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_length * 0.5)

            # Publish the processed image
            img_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            self.publisher_frame.publish(img_msg)
        else:
            self.get_logger().info("No markers detected.")
            
def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()