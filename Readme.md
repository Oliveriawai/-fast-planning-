目前是ROS2 foxy版本

colcon build --symlink-install
source install/setup.bash
ros2 launch plan_manage single_run_in_sim.launch.py

要注意的是，
需要安装 sudo apt-get install ros-jazzy-tf2-geometry_msgs
sudo apt-get install -y ros-jazzy-nav-msgs ros-jazzy-std-msgs ros-jazzy-geometry-msgs
