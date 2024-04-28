![IMG_9162](https://github.com/Dongkyu-ryu/ROS2_Best_Camera/assets/162243656/8f1a54c7-7f1c-4d1e-99ef-00d6dd08e59f)


# ROS2_Best_Camera

The `ROS2_Best_Camera` is a ROS2 package designed for handling multiple camera inputs, processing images, and managing video stream data.

It supports various functionalities including image capture, video recording, and applying image filters such as canny edge detection and salt&pepper noise.

![IMG_9163](https://github.com/Dongkyu-ryu/ROS2_Best_Camera/assets/162243656/21e0e483-1811-46fe-afdb-d0f819cabc18)


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
```

