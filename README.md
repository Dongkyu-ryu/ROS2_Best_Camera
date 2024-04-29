![IMG_9162](https://github.com/Dongkyu-ryu/ROS2_Best_Camera/assets/162243656/8f1a54c7-7f1c-4d1e-99ef-00d6dd08e59f)

# ROS2_Best_Camera
The `ROS2_Best_Camera` is a ROS2 package designed for handling multiple camera inputs, processing images, and managing video stream data.

It supports various functionalities including image capture, video recording, and applying image filters such as canny edge detection and salt&pepper noise.

![IMG_9163](https://github.com/Dongkyu-ryu/ROS2_Best_Camera/assets/162243656/21e0e483-1811-46fe-afdb-d0f819cabc18)

---
## Environment
<img src="https://img.shields.io/badge/ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white"> <img src="https://img.shields.io/badge/python-3776AB?style=for-the-badge&logo=python&logoColor=white">
<img src="https://img.shields.io/badge/opencv-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white"> <img src="https://img.shields.io/badge/github-181717?style=for-the-badge&logo=github&logoColor=white"> 

---

## How To Use?
For those of you who are busy, i have prepared instructions for usage first.

Follow these steps to get started with the `ROS2_Best_Camera` package:

### Step 1: Install & Set up

Before building the package, add below code in your bashrc

plz edit ROS_DOMAIN_ID
```bash
alias sb="source ~/.bashrc; echo \"bashrc is reloaded\""
alias humble="source /opt/ros/humble/setup.bash; ros_domain; echo \"ROS2 humble is activated!\""
alias ros_domain="export ROS_DOMAIN_ID=YOUR OWN DOMAIN NUMBER!"
```

Clone the repository :
```ruby
cd ~
mkdir -p best_camera_ws/src
cd ~/best_camera_ws/src
git clone https://github.com/Dongkyu-ryu/ROS2_Best_Camera.git
```

Build the package:
```ruby
cd ~/best_camera_ws
sb
humble
colcon build
source install/local_setup.bash
```

### Step 2: Running the Camera Node
To launch the camera node which includes image streaming and basic processing capabilities:
```ruby
ros2 launch best_camera camera.launch.py
```
Filter Node :
```ruby
ros2 launch best_camera filter.launch.py
```

### Step 3: Activation camera_service_server 
To use capturing and recording services:
```ruby
ros2 run best_camera camera_service_server
```

### Step 4: Capturing Images
To capture an image using the provided service:
```ruby
ros2 service call /capture best_camera_msgs/srv/CaptureFrame "{topic_name: '/camera'}"
```
Also you can use:
```ruby
ros2 service call /capture best_camera_msgs/srv/CaptureFrame "{topic_name: '/canny'}"
ros2 service call /capture best_camera_msgs/srv/CaptureFrame "{topic_name: '/noise'}"
```

### Step 4 - 1: Recording Videos
Start and stop video recording by calling the record service:

Start recording
```ruby
ros2 service call /record best_camera_msgs/srv/RecordFrame "{start: true, topic_name: '/camera'}"
```
Stop recording
```ruby
ros2 service call /record best_camera_msgs/srv/RecordFrame "{start: false, topic_name: '/camera'}"
```

#### You can find images and videos here:
```ruby
cd ~/best_camera_ws/captured_images
cd ~/best_camera_ws/videos
```
---
