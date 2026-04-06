#  项目简介

底层使用 **fast-planner**，在 **Ubuntu 24.04 + ROS2 JAZZY** 上正常运行。  
上层使用的是仿照 **AirHunt** 的模拟语义 + 价值地图。

---

## 相关资源

- **fast-planner 视频教程**（B站）：  
  https://www.bilibili.com/video/BV1dpWEzXEU6/

- **fast-planner 项目地址**（Gitee）：  
  https://gitee.com/lanlanlanbenben/fast_planner_2004

- **AirHunt 相关论文**：  
  *AirHunt: Bridging VLM Semantics and Continuous Planning for Efficient Aerial Object Navigation*

---

## 编译与运行

### fast-planner版本说明
> 目前代码为 ROS2 **Foxy** 版本。

### 编译命令

colcon build --symlink-install

source install/setup.bash

ros2 launch plan_manage single_run_in_sim.launch.py

- **注意：需要提前安装以下依赖**

sudo apt-get install ros-jazzy-tf2-geometry_msgs

sudo apt-get install -y ros-jazzy-nav-msgs ros-jazzy-std-msgs ros-jazzy-geometry-msgs

- **一些在我编译时处理的问题**：
1. ROS 2 包依赖安装（tf2-geometry-msgs, nav-msgs 等）
  
2. NLopt 库安装和 CMake 配置问题
  
3. 多个包的 CMakeLists.txt 修复

- **关于fast-planner的相关话题**：

话题：`/waypoint_generator/waypoints`，消息类型：`nav_msgs::msg::Path`，回调函数：`waypointCallback`（发布目标点）

- **我的airhunt的运行代码**（自行修改）：

python3 ~/ego_planner_ws/src/airhunt_bridge/airhunt_bridge/airhunt_node_final.py --ros-args -p instruction:="find a green trash bin"

