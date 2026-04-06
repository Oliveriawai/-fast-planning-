#简介
底层使用的是fast-planner,在Ubuntu24.04_ROS2_JAZZY上正常运行，上层使用的是仿照airhunt的模拟语义+价值地图
关于fast-planner，这里推荐观看B站UP主【个人移植，非docker，原生ROS2 Jazzy运行fast planner项目】 https://www.bilibili.com/video/BV1dpWEzXEU6/?share_source=copy_web&vd_source=ad4d7b3694a30637b89f3e68e0edcd41
fast-planner项目地址https://gitee.com/lanlanlanbenben/fast_planner_2004
airhunt相关论文：AirHunt: Bridging VLM Semantics and Continuous Planning for Efficient Aerial Object Navigation
#作者相关的注释：
目前是ROS2 foxy版本（最终编译用这三句）
colcon build --symlink-install
source install/setup.bash
ros2 launch plan_manage single_run_in_sim.launch.py
要注意的是，需要安装 （提前安装）
sudo apt-get install ros-jazzy-tf2-geometry_msgs
sudo apt-get install -y ros-jazzy-nav-msgs ros-jazzy-std-msgs ros-jazzy-geometry-msgs
#一些在我编译时处理的问题：
1. ROS 2 包依赖安装（tf2-geometry-msgs, nav-msgs 等）
2. NLopt 库安装和 CMake 配置问题
3. 多个包的 CMakeLists.txt 修复
#关于fast-planner的相关话题：
话题：`/waypoint_generator/waypoints`，消息类型：`nav_msgs::msg::Path`，回调函数：`waypointCallback`（发布目标点）
#我的airhunt的运行代码（自行修改）：
python3 ~/ego_planner_ws/src/airhunt_bridge/airhunt_bridge/airhunt_node_final.py --ros-args -p instruction:="find a green trash bin"
#有问题多问AI
