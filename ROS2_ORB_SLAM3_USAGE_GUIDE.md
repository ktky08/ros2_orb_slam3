# ROS2 ORB-SLAM3 Usage Guide

This guide provides comprehensive instructions for using the ROS2 ORB-SLAM3 package with different types of datasets and scenarios.

## Table of Contents
1. [Prerequisites](#prerequisites)
2. [Installation](#installation)
3. [Using with EuRoC MAV Dataset](#using-with-euroc-mav-dataset)
4. [Using with Other Datasets (KITTI, TUM-VI, etc.)](#using-with-other-datasets-kitti-tum-vi-etc)
5. [Using with Gazebo Live Simulation](#using-with-gazebo-live-simulation)
6. [Using with ROS2 Bag Files](#using-with-ros2-bag-files)
7. [Troubleshooting](#troubleshooting)

## Prerequisites

### System Requirements
- Ubuntu 22.04 LTS (Jammy Jellyfish)
- ROS2 Humble Hawksbill (LTS)
- At least 8GB RAM (16GB recommended)
- Intel i5/AMD Ryzen 5 or better processor

### Required Dependencies
```bash
# Install system dependencies
sudo apt update
sudo apt install -y libeigen3-dev cmake build-essential git

# Install Python dependencies
pip install "numpy<2"  # Important: Use NumPy 1.x for compatibility
```

### Install Pangolin
```bash
# Clone and build Pangolin
cd ~/Documents
git clone https://github.com/stevenlovegrove/Pangolin
cd Pangolin
./scripts/install_prerequisites.sh recommended
cmake -B build
cmake --build build -j4
sudo cmake --install build

# Configure dynamic library path
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
sudo ldconfig

# Add to .bashrc for persistence
echo 'if [[ ":$LD_LIBRARY_PATH:" != *":/usr/local/lib:"* ]]; then export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH; fi' >> ~/.bashrc
source ~/.bashrc
```

## Installation

### 1. Clone the Repository
```bash
cd ~/ros2_ws/src
git clone https://github.com/ktky08/ros2_orb_slam3.git
```

### 2. Fix Hardcoded Paths
The package contains hardcoded paths that need to be updated for your workspace:

**Update Python script path:**
```bash
# Edit: src/ros2_orb_slam3/scripts/mono_driver_node.py
# Line 73: Change from "ros2_test" to "ros2_ws"
self.home_dir = str(Path.home()) + "/ros2_ws/src/ros2_orb_slam3"
```

**Update C++ header path:**
```bash
# Edit: src/ros2_orb_slam3/include/ros2_orb_slam3/common.hpp
# Line 71: Change from "ros2_test" to "ros2_ws"
std::string packagePath = "ros2_ws/src/ros2_orb_slam3/";
```

### 3. Build the Package
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ros2_orb_slam3
source ./install/setup.bash
```

## Using with EuRoC MAV Dataset

The package comes with a sample EuRoC MAV dataset for testing.

### 1. Run the System
```bash
# Terminal 1: Start the C++ SLAM node
cd ~/ros2_ws
source ./install/setup.bash
ros2 run ros2_orb_slam3 mono_node_cpp --ros-args -p node_name_arg:=mono_slam_cpp

# Terminal 2: Start the Python driver node
cd ~/ros2_ws
source ./install/setup.bash
ros2 run ros2_orb_slam3 mono_driver_node.py --ros-args -p settings_name:=EuRoC -p image_seq:=sample_euroc_MH05
```

### 2. Expected Output
- Python node loads 585 images from the test dataset
- C++ node initializes ORB-SLAM3 with camera parameters
- Handshake completes between nodes
- SLAM system starts processing images and building maps
- Keyframes are created and 3D points are mapped

### 3. Using Your Own EuRoC Dataset
To use your own EuRoC MAV dataset:

1. **Prepare your dataset** in the EuRoC format:
   ```
   your_dataset/
   ├── mav0/
   │   ├── cam0/
   │   │   ├── data/
   │   │   │   ├── 1403638567777829376.png
   │   │   │   ├── 1403638567727829504.png
   │   │   │   └── ...
   │   │   ├── data.csv
   │   │   └── sensor.yaml
   │   └── state_groundtruth_estimate0/
   ```

2. **Place your dataset** in the TEST_DATASET directory:
   ```bash
   cp -r your_dataset ~/ros2_ws/src/ros2_orb_slam3/TEST_DATASET/your_dataset_name
   ```

3. **Run with your dataset**:
   ```bash
   ros2 run ros2_orb_slam3 mono_driver_node.py --ros-args -p settings_name:=EuRoC -p image_seq:=your_dataset_name
   ```

## Using with Other Datasets (KITTI, TUM-VI, etc.)

The ORB-SLAM3 package includes configuration files for multiple popular datasets. You can use these configurations with your own dataset files.

### Available Dataset Configurations

The package includes configurations for:

**Monocular Mode:**
- `EuRoC.yaml` - EuRoC MAV dataset
- `KITTI00-02.yaml`, `KITTI03.yaml`, `KITTI04-12.yaml` - KITTI dataset sequences
- `TUM1.yaml`, `TUM2.yaml`, `TUM3.yaml` - TUM RGB-D dataset
- `TUM-VI.yaml` - TUM Visual-Inertial dataset
- `RealSense_D435i.yaml`, `RealSense_T265.yaml` - Intel RealSense cameras
- `NTU_VIRAL.yaml` - NTU VIRAL dataset
- `GazeboGarden.yaml` - Gazebo Garden simulation

**Stereo Mode:**
- `EuRoC.yaml` - EuRoC MAV dataset (stereo)
- `KITTI00-02.yaml`, `KITTI03.yaml`, `KITTI04-12.yaml` - KITTI dataset (stereo)
- `TUM-VI.yaml` - TUM Visual-Inertial dataset (stereo)
- `RealSense_D435i.yaml`, `RealSense_T265.yaml` - Intel RealSense cameras (stereo)

**Monocular-Inertial Mode:**
- `EuRoC.yaml` - EuRoC MAV dataset (with IMU)
- `TUM-VI.yaml` - TUM Visual-Inertial dataset (with IMU)
- `RealSense_D435i.yaml`, `RealSense_T265.yaml` - Intel RealSense cameras (with IMU)

### Using KITTI Dataset

1. **Download KITTI Dataset:**
   ```bash
   # Download from KITTI website: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
   # Extract to: ~/ros2_ws/src/ros2_orb_slam3/TEST_DATASET/kitti_sequence_XX/
   ```

2. **Prepare Your Dataset:**
   ```bash
   # Create directory structure
   mkdir -p ~/ros2_ws/src/ros2_orb_slam3/TEST_DATASET/kitti_sequence_00
   
   # Copy images to the directory
   # KITTI format: image_2/000000.png, image_2/000001.png, etc.
   cp /path/to/kitti/sequences/00/image_2/*.png ~/ros2_ws/src/ros2_orb_slam3/TEST_DATASET/kitti_sequence_00/
   ```

3. **Run with KITTI Configuration:**
   ```bash
   # Terminal 1: Start the C++ SLAM node
   cd ~/ros2_ws
   source ./install/setup.bash
   ros2 run ros2_orb_slam3 mono_node_cpp --ros-args -p node_name_arg:=mono_slam_cpp

   # Terminal 2: Start the Python driver node
   cd ~/ros2_ws
   source ./install/setup.bash
   ros2 run ros2_orb_slam3 mono_driver_node.py --ros-args -p settings_name:=KITTI00-02 -p image_seq:=kitti_sequence_00
   ```

### Using TUM-VI Dataset

1. **Download TUM-VI Dataset:**
   ```bash
   # Download from TUM website: https://vision.in.tum.de/data/datasets/visual-inertial-dataset
   # Extract to: ~/ros2_ws/src/ros2_orb_slam3/TEST_DATASET/tum_vi_room1/
   ```

2. **Prepare Your Dataset:**
   ```bash
   # Create directory structure
   mkdir -p ~/ros2_ws/src/ros2_orb_slam3/TEST_DATASET/tum_vi_room1
   
   # Copy images to the directory
   # TUM-VI format: cam0/1403715282262142976.png, cam0/1403715282312143104.png, etc.
   cp /path/to/tum_vi/dataset-room1_512_16/cam0/*.png ~/ros2_ws/src/ros2_orb_slam3/TEST_DATASET/tum_vi_room1/
   ```

3. **Run with TUM-VI Configuration:**
   ```bash
   # Terminal 1: Start the C++ SLAM node
   cd ~/ros2_ws
   source ./install/setup.bash
   ros2 run ros2_orb_slam3 mono_node_cpp --ros-args -p node_name_arg:=mono_slam_cpp

   # Terminal 2: Start the Python driver node
   cd ~/ros2_ws
   source ./install/setup.bash
   ros2 run ros2_orb_slam3 mono_driver_node.py --ros-args -p settings_name:=TUM-VI -p image_seq:=tum_vi_room1
   ```

### Using Custom Datasets

To use your own dataset with any of the provided configurations:

1. **Choose the appropriate configuration file** based on your camera parameters:
   - Check the camera resolution, FOV, and calibration parameters
   - Match your dataset to the closest configuration

2. **Prepare your image sequence:**
   ```bash
   # Create directory for your dataset
   mkdir -p ~/ros2_ws/src/ros2_orb_slam3/TEST_DATASET/my_dataset
   
   # Copy your images (PNG or JPG format)
   cp /path/to/your/images/*.png ~/ros2_ws/src/ros2_orb_slam3/TEST_DATASET/my_dataset/
   ```

3. **Run with the chosen configuration:**
   ```bash
   # Terminal 1: Start the C++ SLAM node
   cd ~/ros2_ws
   source ./install/setup.bash
   ros2 run ros2_orb_slam3 mono_node_cpp --ros-args -p node_name_arg:=mono_slam_cpp

   # Terminal 2: Start the Python driver node
   cd ~/ros2_ws
   source ./install/setup.bash
   ros2 run ros2_orb_slam3 mono_driver_node.py --ros-args -p settings_name:=EuRoC -p image_seq:=my_dataset
   ```

### Configuration File Details

Each configuration file contains:

- **Camera Parameters:** Intrinsic calibration (fx, fy, cx, cy), distortion coefficients
- **Image Settings:** Resolution, FPS, color format
- **ORB Parameters:** Number of features, scale factors, FAST thresholds
- **Viewer Parameters:** Visualization settings

**Example KITTI Configuration:**
```yaml
Camera1.fx: 718.856
Camera1.fy: 718.856
Camera1.cx: 607.1928
Camera1.cy: 185.2157
Camera.width: 1241
Camera.height: 376
Camera.fps: 10
```

**Example TUM-VI Configuration:**
```yaml
Camera1.fx: 190.978477
Camera1.fy: 190.973307
Camera1.cx: 254.931706
Camera1.cy: 256.897442
Camera.width: 512
Camera.height: 512
Camera.fps: 20
```

### Tips for Dataset Selection

1. **Camera Resolution:** Choose a configuration that matches your image resolution
2. **Frame Rate:** Higher FPS configurations work better with fast-moving sequences
3. **Camera Type:** 
   - `PinHole` for standard cameras (KITTI, EuRoC)
   - `KannalaBrandt8` for fisheye cameras (TUM-VI)
4. **Feature Count:** Adjust `ORBextractor.nFeatures` based on your scene complexity

### Installed Test Datasets

The package now includes the following test datasets ready for use:

**Available Datasets:**
- **EuRoC MH05**: `sample_euroc_MH05` (original sample)
- **KITTI Sequence 00**: `kitti_sequence_00` (585 images, KITTI-style)
- **TUM-VI Room1**: `tum_vi_room1` (585 images, TUM-VI-style)

**Dataset Locations:**
```bash
~/ros2_ws/src/ros2_orb_slam3/TEST_DATASET/
├── sample_euroc_MH05/     # Original EuRoC sample
├── kitti_sequence_00/     # KITTI-style test dataset
└── tum_vi_room1/         # TUM-VI-style test dataset
```

**Quick Test Commands:**
```bash
# Test KITTI dataset
python3 ~/ros2_ws/install/ros2_orb_slam3/lib/ros2_orb_slam3/test_dataset.py KITTI00-02 kitti_sequence_00

# Test TUM-VI dataset  
python3 ~/ros2_ws/install/ros2_orb_slam3/lib/ros2_orb_slam3/test_dataset.py TUM-VI tum_vi_room1

# Test EuRoC dataset
python3 ~/ros2_ws/install/ros2_orb_slam3/lib/ros2_orb_slam3/test_dataset.py EuRoC sample_euroc_MH05
```

### Quick Dataset Testing

For quick testing of different datasets, you can use the `test_dataset.py` script:

```bash
# Test with EuRoC dataset (default)
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/install/ros2_orb_slam3/lib/ros2_orb_slam3/test_dataset.py EuRoC sample_euroc_MH05

# Test with KITTI configuration
python3 ~/ros2_ws/install/ros2_orb_slam3/lib/ros2_orb_slam3/test_dataset.py KITTI00-02 kitti_sequence_00

# Test with TUM-VI configuration
python3 ~/ros2_ws/install/ros2_orb_slam3/lib/ros2_orb_slam3/test_dataset.py TUM-VI tum_vi_room1

# Test with custom dataset
python3 ~/ros2_ws/install/ros2_orb_slam3/lib/ros2_orb_slam3/test_dataset.py EuRoC my_custom_dataset
```

**Usage:**
```bash
python3 ~/ros2_ws/install/ros2_orb_slam3/lib/ros2_orb_slam3/test_dataset.py [dataset_type] [sequence_name]
```

**Parameters:**
- `dataset_type`: Configuration file name (e.g., EuRoC, KITTI00-02, TUM-VI)
- `sequence_name`: Directory name in TEST_DATASET folder

**Note:** The script automatically:
- Loads images from the specified dataset directory
- Publishes them at 10 FPS to the ORB-SLAM3 system
- Shows progress updates every 50 frames
- Completes when all images are processed

## Using with Gazebo Garden Live Simulation

### Prerequisites
- Gazebo Garden (gz sim 8.9.0 or later) installed
- PX4 SITL (Software In The Loop) setup
- Camera sensor configured in your Gazebo world
- Camera calibration parameters (intrinsic matrix, distortion coefficients)

### Setup Steps

1. **Start Gazebo Garden with your custom world:**
```bash
# For your specific setup with custom forest world
PX4_GZ_WORLD=z_my_forest make px4_sitl gz_x500_custom

# Alternative: Direct gz sim command
gz sim -s -v 4 z_my_forest.sdf
```

2. **In a new terminal, start the ORB-SLAM3 C++ node:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run ros2_orb_slam3 mono_node_cpp --ros-args -p node_name_arg:=mono_slam_cpp
```

3. **In another terminal, start the Gazebo Garden driver:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run ros2_orb_slam3 gazebo_garden_driver.py --ros-args -p camera_topic:=/world/z_my_forest/model/x500_custom_0/link/camera_link/sensor/camera/image
```

### Camera Configuration

**Important:** ORB-SLAM3 requires camera calibration parameters to work properly. The package includes a custom configuration for Gazebo Garden:

**Default Configuration File:** `orb_slam3/config/Monocular/GazeboGarden.yaml`

**Key Camera Parameters:**
- **Focal Length:** fx=500.0, fy=500.0 (pixels)
- **Principal Point:** cx=320.0, cy=240.0 (image center)
- **Resolution:** 640x480 pixels
- **Distortion:** Minimal (k1=k2=p1=p2=0.0 for simulated camera)

**Customizing Camera Parameters:**
If your camera has different parameters, edit the configuration file:
```bash
# Edit the camera configuration
nano ~/ros2_ws/src/ros2_orb_slam3/orb_slam3/config/Monocular/GazeboGarden.yaml
```

**Finding Your Camera Parameters:**
1. **For real cameras:** Use camera calibration tools (e.g., `camera_calibration` package)
2. **For Gazebo:** Check your camera model's SDF/URDF file for intrinsic parameters
3. **For unknown cameras:** Start with the default values and adjust based on performance

### Configuration Options

**Camera Topic Parameters:**
- Default: `/world/z_my_forest/model/x500_custom_0/link/camera_link/sensor/camera/image`
- Camera Info: `/world/z_my_forest/model/x500_custom_0/link/camera_link/sensor/camera/camera_info`
- Settings: `EuRoC` (default)

**Custom Camera Topics:**
If your camera has a different topic name, you can specify it:
```bash
ros2 run ros2_orb_slam3 gazebo_garden_driver.py --ros-args \
  -p camera_topic:=/your_camera_topic \
  -p settings_name:=EuRoC
```

**Finding Your Camera Topic:**
```bash
# List all available topics
ros2 topic list | grep camera

# For Gazebo Garden with ROS2 bridge, camera topics typically follow this pattern:
# /world/{world_name}/model/{model_name}_{instance}/link/camera_link/sensor/camera/image

# Monitor camera topics
ros2 topic echo /world/z_my_forest/model/x500_custom_0/link/camera_link/sensor/camera/image --once
```

### Troubleshooting Gazebo Garden

**Common Issues:**
1. **Camera topic not found**: Use `ros2 topic list` to find the correct camera topic
2. **No images received**: Check if the camera sensor is properly configured in your world
3. **Permission issues**: Ensure your user has access to the camera device
4. **Poor SLAM performance**: Check camera calibration parameters
5. **No features detected**: Adjust ORB parameters or check image quality

**Camera Calibration Issues:**
- **Blurry tracking:** Increase `ORBextractor.iniThFAST` and `ORBextractor.minThFAST`
- **Too few features:** Increase `ORBextractor.nFeatures` or improve lighting
- **Scale issues:** Verify focal length (fx, fy) matches your camera
- **Distortion problems:** Check distortion coefficients (k1, k2, p1, p2)

**Verification Steps:**
```bash
# Check if Gazebo is publishing camera topics
ros2 topic list | grep -i camera

# Check camera topic frequency
ros2 topic hz /world/z_my_forest/model/x500_custom_0/link/camera_link/sensor/camera/image

# View camera images (optional)
ros2 run rqt_image_view rqt_image_view
```

### Alternative: Using Live Camera Driver

If you want to use a real camera instead of Gazebo, you can use the `live_camera_driver.py`:

```bash
# Terminal 1: Start ORB-SLAM3 C++ node
source ~/ros2_ws/install/setup.bash
ros2 run ros2_orb_slam3 mono_node_cpp --ros-args -p node_name_arg:=mono_slam_cpp

# Terminal 2: Start live camera driver
source ~/ros2_ws/install/setup.bash
ros2 run ros2_orb_slam3 live_camera_driver.py
```

## Using with ROS2 Bag Files

### 1. Record a ROS2 Bag
```bash
# Record camera topics
ros2 bag record /camera/image_raw /camera/camera_info

# Or record all topics
ros2 bag record -a
```

### 2. Create a Bag File Driver
Create a new file: `~/ros2_ws/src/ros2_orb_slam3/scripts/bag_driver.py`

```python
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
```

### 3. Run with Bag File
```bash
# Terminal 1: Play the bag file
ros2 bag play your_bag_file

# Terminal 2: Run C++ SLAM node
ros2 run ros2_orb_slam3 mono_node_cpp --ros-args -p node_name_arg:=mono_slam_cpp

# Terminal 3: Run bag driver
ros2 run ros2_orb_slam3 bag_driver.py --ros-args -p bag_file_path:=/path/to/your/bag/file -p image_topic:=/camera/image_raw
```

## Troubleshooting

### Common Issues

#### 1. NumPy Compatibility Error
```
ImportError: numpy.core.multiarray failed to import
```
**Solution:**
```bash
pip install "numpy<2"
```

#### 2. Package Not Found
```
Package 'ros2_orb_slam3' not found
```
**Solution:**
```bash
cd ~/ros2_ws
source ./install/setup.bash
```

#### 3. Hardcoded Path Errors
```
Failed to open settings file at: /home/ken/ros2_test/...
```
**Solution:** Update the hardcoded paths as described in the Installation section.

#### 4. Pangolin Library Not Found
```
error while loading shared libraries: libpango_*.so
```
**Solution:**
```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
sudo ldconfig
```

#### 5. Camera Not Found
```
Failed to open camera
```
**Solution:**
- Check camera permissions: `sudo usermod -a -G video $USER`
- Verify camera device: `ls /dev/video*`
- Test with: `v4l2-ctl --list-devices`

### Performance Tips

1. **Reduce Image Resolution** for better performance
2. **Adjust ORB Parameters** in the configuration file
3. **Use SSD Storage** for faster data access
4. **Close Unnecessary Applications** to free up memory

### Debugging

#### Enable Debug Output
```bash
# Set ROS log level
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_LEVEL=DEBUG
```

#### Monitor System Resources
```bash
# Monitor CPU and memory
htop

# Monitor disk I/O
iotop

# Monitor network (if using remote cameras)
iftop
```

## Advanced Configuration

### Custom Camera Parameters
Edit the camera configuration file:
```bash
nano ~/ros2_ws/src/ros2_orb_slam3/orb_slam3/config/Monocular/EuRoC.yaml
```

Key parameters:
- `Camera.fx`, `Camera.fy`: Focal length
- `Camera.cx`, `Camera.cy`: Principal point
- `Camera.k1`, `Camera.k2`: Distortion coefficients
- `ORBextractor.nFeatures`: Number of ORB features
- `ORBextractor.scaleFactor`: Scale factor between levels

### Multi-Camera Setup
For stereo or multi-camera setups, modify the driver scripts to handle multiple camera streams and use the appropriate ORB-SLAM3 mode (stereo, RGB-D).

## Contributing

When contributing to this project:
1. Follow the existing code style
2. Test with different datasets
3. Update documentation
4. Add appropriate error handling
5. Include performance benchmarks

## License

This project is based on ORB-SLAM3 which is licensed under GPL-3.0. See the LICENSE file for details.
