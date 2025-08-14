#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String
from cv_bridge import CvBridge
import cv2
import time

class LiveCameraDriver(Node):
    def __init__(self):
        super().__init__('live_camera_driver')
        
        # Publishers
        self.img_publisher = self.create_publisher(Image, '/mono_py_driver/img_msg', 10)
        self.timestep_publisher = self.create_publisher(Float64, '/mono_py_driver/timestep_msg', 10)
        self.config_publisher = self.create_publisher(String, '/mono_py_driver/experiment_settings', 10)
        
        # Subscriber for handshake
        self.ack_subscriber = self.create_subscription(
            String, '/mono_py_driver/exp_settings_ack', self.ack_callback, 10)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Camera setup
        self.cap = cv2.VideoCapture(0)  # Use 0 for default camera, or specify device
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera')
            return
            
        # Timer for publishing images
        self.timer = self.create_timer(0.1, self.publish_frame)  # 10 FPS
        
        # Handshake
        self.handshake_complete = False
        self.send_handshake()
        
        self.get_logger().info('Live camera driver initialized')
    
    def send_handshake(self):
        config_msg = String()
        config_msg.data = "EuRoC"
        self.config_publisher.publish(config_msg)
        self.get_logger().info('Sent handshake request')
    
    def ack_callback(self, msg):
        if msg.data == "ACK":
            self.handshake_complete = True
            self.get_logger().info('Handshake complete!')
    
    def publish_frame(self):
        if not self.handshake_complete:
            return
            
        ret, frame = self.cap.read()
        if ret:
            # Convert to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            
            # Publish image
            self.img_publisher.publish(ros_image)
            
            # Publish timestep
            timestep_msg = Float64()
            timestep_msg.data = time.time()
            self.timestep_publisher.publish(timestep_msg)
    
    def __del__(self):
        if hasattr(self, 'cap'):
            self.cap.release()

def main():
    rclpy.init()
    node = LiveCameraDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
