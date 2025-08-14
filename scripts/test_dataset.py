#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String
from cv_bridge import CvBridge
import cv2
import time
import os
import sys
from pathlib import Path

class DatasetTester(Node):
    def __init__(self, dataset_type='EuRoC', sequence_name='sample_euroc_MH05'):
        super().__init__('dataset_tester')
        
        # Use provided parameters instead of declaring ROS parameters
        self.dataset_type = dataset_type
        self.sequence_name = sequence_name
        self.image_rate = 10.0  # Hz
        
        # Publishers
        self.img_publisher = self.create_publisher(Image, '/mono_py_driver/img_msg', 10)
        self.timestep_publisher = self.create_publisher(Float64, '/mono_py_driver/timestep_msg', 10)
        self.config_publisher = self.create_publisher(String, '/mono_py_driver/experiment_settings', 10)
        
        # Subscriber for handshake
        self.ack_subscriber = self.create_subscription(
            String, '/mono_py_driver/exp_settings_ack', self.ack_callback, 10)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Setup paths
        self.home_dir = str(Path.home()) + "/ros2_ws/src/ros2_orb_slam3"
        self.dataset_dir = self.home_dir + "/TEST_DATASET/" + self.sequence_name
        
        # Check if dataset exists
        if not os.path.exists(self.dataset_dir):
            self.get_logger().error(f'Dataset directory not found: {self.dataset_dir}')
            self.get_logger().error('Please ensure your dataset is in the correct location')
            return
        
        # Get image files
        self.image_files = self.get_image_files()
        if not self.image_files:
            self.get_logger().error(f'No image files found in: {self.dataset_dir}')
            return
        
        self.get_logger().info(f'Found {len(self.image_files)} images in dataset')
        
        # State variables
        self.handshake_complete = False
        self.current_image_index = 0
        self.start_time = time.time()
        
        # Send handshake
        self.send_handshake()
        
        # Timer for publishing images
        timer_period = 1.0 / self.image_rate
        self.timer = self.create_timer(timer_period, self.publish_next_image)
        
        self.get_logger().info(f'Dataset tester initialized for {self.dataset_type} - {self.sequence_name}')
    
    def get_image_files(self):
        """Get list of image files from dataset directory based on dataset type"""
        image_extensions = ['.png', '.jpg', '.jpeg', '.bmp', '.tiff']
        image_files = []
        
        # Handle different dataset structures
        if self.dataset_type == 'KITTI00-02' or self.dataset_type == 'KITTI03' or self.dataset_type == 'KITTI04-12':
            # KITTI format: image_2/000000.png, image_2/000001.png, etc.
            image_dir = os.path.join(self.dataset_dir, 'image_2')
            if os.path.exists(image_dir):
                for file in os.listdir(image_dir):
                    if any(file.lower().endswith(ext) for ext in image_extensions):
                        image_files.append(os.path.join(image_dir, file))
            else:
                # Fallback: look directly in dataset directory
                for file in os.listdir(self.dataset_dir):
                    if any(file.lower().endswith(ext) for ext in image_extensions):
                        image_files.append(os.path.join(self.dataset_dir, file))
        
        elif self.dataset_type == 'TUM-VI':
            # TUM-VI format: cam0/1403715282262142976.png, cam0/1403715282312143104.png, etc.
            image_dir = os.path.join(self.dataset_dir, 'cam0')
            if os.path.exists(image_dir):
                for file in os.listdir(image_dir):
                    if any(file.lower().endswith(ext) for ext in image_extensions):
                        image_files.append(os.path.join(image_dir, file))
            else:
                # Fallback: look directly in dataset directory
                for file in os.listdir(self.dataset_dir):
                    if any(file.lower().endswith(ext) for ext in image_extensions):
                        image_files.append(os.path.join(self.dataset_dir, file))
        
        elif self.dataset_type == 'EuRoC':
            # EuRoC format: mav0/cam0/data/1403638567777829376.png, etc.
            image_dir = os.path.join(self.dataset_dir, 'mav0', 'cam0', 'data')
            if os.path.exists(image_dir):
                for file in os.listdir(image_dir):
                    if any(file.lower().endswith(ext) for ext in image_extensions):
                        image_files.append(os.path.join(image_dir, file))
            else:
                # Fallback: look directly in dataset directory
                for file in os.listdir(self.dataset_dir):
                    if any(file.lower().endswith(ext) for ext in image_extensions):
                        image_files.append(os.path.join(self.dataset_dir, file))
        
        else:
            # Default: look directly in dataset directory
            for file in os.listdir(self.dataset_dir):
                if any(file.lower().endswith(ext) for ext in image_extensions):
                    image_files.append(os.path.join(self.dataset_dir, file))
        
        # Sort files to ensure consistent order
        image_files.sort()
        return image_files
    
    def send_handshake(self):
        config_msg = String()
        config_msg.data = self.dataset_type
        self.config_publisher.publish(config_msg)
        self.get_logger().info(f'Sent handshake request for {self.dataset_type}')
    
    def ack_callback(self, msg):
        if msg.data == "ACK":
            self.handshake_complete = True
            self.get_logger().info('Handshake complete! Starting dataset playback...')
    
    def publish_next_image(self):
        if not self.handshake_complete:
            return
        
        if self.current_image_index >= len(self.image_files):
            self.get_logger().info('Dataset playback complete!')
            self.timer.cancel()
            return
        
        try:
            # Load and publish image
            image_path = self.image_files[self.current_image_index]
            image = cv2.imread(image_path)
            
            if image is not None:
                # Convert to ROS Image message
                ros_image = self.bridge.cv2_to_imgmsg(image, "bgr8")
                
                # Publish image
                self.img_publisher.publish(ros_image)
                
                # Publish timestep
                timestep_msg = Float64()
                timestep_msg.data = time.time()
                self.timestep_publisher.publish(timestep_msg)
                
                # Log progress
                if self.current_image_index % 50 == 0:
                    progress = (self.current_image_index / len(self.image_files)) * 100
                    self.get_logger().info(f'Progress: {progress:.1f}% ({self.current_image_index}/{len(self.image_files)})')
                
                self.current_image_index += 1
            else:
                self.get_logger().warn(f'Failed to load image: {image_path}')
                self.current_image_index += 1
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
            self.current_image_index += 1

def main():
    rclpy.init()
    
    # Check command line arguments
    if len(sys.argv) > 1:
        dataset_type = sys.argv[1]
    else:
        dataset_type = 'EuRoC'
    
    if len(sys.argv) > 2:
        sequence_name = sys.argv[2]
    else:
        sequence_name = 'sample_euroc_MH05'
    
    # Create node with parameters
    node = DatasetTester(dataset_type, sequence_name)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Dataset testing interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
