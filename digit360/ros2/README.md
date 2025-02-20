
# Digit 360 Interfaces
This guide will help you set up the environment, build the ROS2 package, and join the growing community of developers and researchers.

## 🛠 Virtual Environment Setup

To install and activate the virtual environment for Digit 360, run the following commands:

```
cd digit360
./digit360/install_digit360_venv.sh
```
This will handle the necessary dependencies and ensure your environment is ready for development.

## 🛠 Build ROS2 pkg
```
mkdir -p digit360_ws/src
cd digit360_ws/src
git clone https://github.com/facebookresearch/digit360.git
cd ..
sh ./src/digit360/digit360/ros2/d360/colcon_build.sh
```

## 🛠 Install digit360 pkg
```
cd digit360_ws/src/digit360/digit360/
pip install -e .
```

## Source the ws

```
cd digit360_ws
source install/setup.bash
```

This creates the workspace, builds the source, and prepares everything for use with ROS2 nodes and topics.

## 🚀 Run Commands 
To launch the Digit 360 package (d360), use the following command:

This launches the minimal version of d360 - image, audio, and multimodal data topics,
```bash
ros2 launch d360 d360_min_launch.py
```

To use with a Foxglove for visualizing data (foxglove+d360 minimal) version,
```bash
ros2 launch d360 d360_launch.py
```

### Sensor and Multimodal Specific Launches

For audio topics only,
```bash
 ros2 run d360 audio_pub
```

For temporal multimodal only,

```bash
 ros2 run d360 sensors_pub
```

For image and visotactile modalities only,
```bash
ros2 run d360 image_pub_opencv
```
