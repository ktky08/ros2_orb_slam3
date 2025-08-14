#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String
from cv_bridge import CvBridge
import cv2
import time
import os

class BagFileDriver(Node):
    def __init__(self):
        super().__init__('bag_file_driver')
        
        # Parameters
        self.declare_parameter('bag_file_path', '')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('playback_rate', 1.0)
        
        bag_path = self.get_parameter('bag_file_path').value
        image_topic = self.get_parameter('image_topic').value
        self.playback_rate = self.get_parameter('playback_rate').value
        
        if not bag_path or not os.path.exists(bag_path):
            self.get_logger().error(f'Bag file not found: {bag_path}')
            return
        
        # Publishers
        self.img_publisher = self.create_publisher(Image, '/mono_py_driver/img_msg', 10)
        self.timestep_publisher = self.create_publisher(Float64, '/mono_py_driver/timestep_msg', 10)
        self.config_publisher = self.create_publisher(String, '/mono_py_driver/experiment_settings', 10)
        
        # Subscriber for handshake
        self.ack_subscriber = self.create_subscription(
            String, '/mono_py_driver/exp_settings_ack', self.ack_callback, 10)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Handshake
        self.handshake_complete = False
        self.send_handshake()
        
        # Subscribe to bag file topics
        self.image_subscriber = self.create_subscription(
            Image, image_topic, self.image_callback, 10)
        
        self.get_logger().info(f'Bag file driver initialized for: {bag_path}')
    
    def send_handshake(self):
        config_msg = String()
        config_msg.data = "EuRoC"
        self.config_publisher.publish(config_msg)
        self.get_logger().info('Sent handshake request')
    
    def ack_callback(self, msg):
        if msg.data == "ACK":
            self.handshake_complete = True
            self.get_logger().info('Handshake complete!')
    
    def image_callback(self, msg):
        if not self.handshake_complete:
            return
        
        # Publish image to SLAM node
        self.img_publisher.publish(msg)
        
        # Publish timestep
        timestep_msg = Float64()
        timestep_msg.data = time.time()
        self.timestep_publisher.publish(timestep_msg)

def main():
    rclpy.init()
    node = BagFileDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
