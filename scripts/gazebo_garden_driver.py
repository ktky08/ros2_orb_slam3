#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String
from cv_bridge import CvBridge
import cv2
import time

class GazeboGardenDriver(Node):
    def __init__(self):
        super().__init__('gazebo_garden_driver')
        
        # Parameters for Gazebo Garden
        self.declare_parameter('camera_topic', '/world/z_my_forest/model/x500_custom_0/link/camera_link/sensor/camera/image')
        self.declare_parameter('camera_info_topic', '/world/z_my_forest/model/x500_custom_0/link/camera_link/sensor/camera/camera_info')
        self.declare_parameter('settings_name', 'EuRoC')
        
        camera_topic = self.get_parameter('camera_topic').value
        settings_name = self.get_parameter('settings_name').value
        
        # Publishers for ORB-SLAM3
        self.img_publisher = self.create_publisher(Image, '/mono_py_driver/img_msg', 10)
        self.timestep_publisher = self.create_publisher(Float64, '/mono_py_driver/timestep_msg', 10)
        self.config_publisher = self.create_publisher(String, '/mono_py_driver/experiment_settings', 10)
        
        # Subscribers for Gazebo Garden
        self.camera_subscriber = self.create_subscription(
            Image, camera_topic, self.camera_callback, 10)
        
        # Subscriber for handshake
        self.ack_subscriber = self.create_subscription(
            String, '/mono_py_driver/exp_settings_ack', self.ack_callback, 10)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # State variables
        self.handshake_complete = False
        self.frame_count = 0
        self.start_time = time.time()
        
        # Send handshake
        self.send_handshake()
        
        self.get_logger().info(f'Gazebo Garden driver initialized')
        self.get_logger().info(f'Subscribing to camera topic: {camera_topic}')
        self.get_logger().info(f'Waiting for camera images from Gazebo...')
    
    def send_handshake(self):
        config_msg = String()
        config_msg.data = self.get_parameter('settings_name').value
        self.config_publisher.publish(config_msg)
        self.get_logger().info('Sent handshake request')
    
    def ack_callback(self, msg):
        if msg.data == "ACK":
            self.handshake_complete = True
            self.get_logger().info('Handshake complete! ORB-SLAM3 is ready to receive images.')
    
    def camera_callback(self, msg):
        if not self.handshake_complete:
            self.get_logger().warn('Received camera image but handshake not complete yet')
            return
        
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Publish to ORB-SLAM3
            self.img_publisher.publish(msg)
            
            # Publish timestep
            timestep_msg = Float64()
            timestep_msg.data = time.time()
            self.timestep_publisher.publish(timestep_msg)
            
            self.frame_count += 1
            if self.frame_count % 30 == 0:  # Log every 30 frames
                elapsed_time = time.time() - self.start_time
                fps = self.frame_count / elapsed_time
                self.get_logger().info(f'Processed {self.frame_count} frames, FPS: {fps:.2f}')
                
        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {str(e)}')

def main():
    rclpy.init()
    node = GazeboGardenDriver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Gazebo Garden driver...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
