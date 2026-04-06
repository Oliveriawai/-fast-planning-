#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np
import threading
import queue
import time
import math

# ========== 配置 ==========
GOAL_TOPIC = "/waypoint_generator/waypoints"
MAP_WIDTH = 30.0
MAP_HEIGHT = 30.0
RESOLUTION = 0.5

# 目标区域中心（模拟"绿色垃圾桶"的位置）
TARGET_X = 12.0
TARGET_Y = 12.0


# ========== 价值地图 ==========
class ValueMap3D:
    def __init__(self, width, height, resolution):
        self.width = width
        self.height = height
        self.res = resolution
        nx = int(width / resolution) + 1
        ny = int(height / resolution) + 1
        self.grid = np.zeros((nx, ny))
        self.conf = np.zeros((nx, ny))
    
    def xy_to_idx(self, x, y):
        ix = int(x / self.res)
        iy = int(y / self.res)
        ix = max(0, min(ix, self.grid.shape[0] - 1))
        iy = max(0, min(iy, self.grid.shape[1] - 1))
        return ix, iy
    
    def update(self, x, y, value, confidence=0.7):
        ix, iy = self.xy_to_idx(x, y)
        old_v = self.grid[ix, iy]
        old_c = self.conf[ix, iy]
        new_v = (old_c * old_v + confidence * value) / (old_c + confidence + 1e-6)
        new_c = min(1.0, old_c + confidence)
        self.grid[ix, iy] = new_v
        self.conf[ix, iy] = new_c
    
    def get_best_target(self):
        idx = np.unravel_index(np.argmax(self.grid), self.grid.shape)
        x = idx[0] * self.res
        y = idx[1] * self.res
        return np.array([x, y, 1.5]), self.grid[idx]


# ========== VLM 模拟（基于距离目标区域的距离）==========
class MockVLM:
    def infer(self, instruction, pos):
        """根据与目标区域的距离返回语义值"""
        x, y = pos[0], pos[1]
        # 目标区域在 (TARGET_X, TARGET_Y)
        dist_to_target = math.sqrt((x - TARGET_X)**2 + (y - TARGET_Y)**2)
        # 距离越近，价值越高
        value = 0.9 * math.exp(-dist_to_target / 3.0)
        return min(max(value, 0.1), 0.95)


# ========== AirHunt ROS2 节点 ==========
class AirHuntNode(Node):
    def __init__(self):
        super().__init__('airhunt_node')
        
        # 参数
        self.declare_parameter('instruction', 'find the green trash bin on the roadside')
        self.instruction = self.get_parameter('instruction').value
        
        # 初始化
        self.vlm = MockVLM()
        self.value_map = ValueMap3D(MAP_WIDTH, MAP_HEIGHT, RESOLUTION)
        self.current_pos = [0.0, 0.0, 1.5]
        self.found = False
        self.step = 0
        
        # 目标发布器
        self.goal_pub = self.create_publisher(Path, GOAL_TOPIC, 10)
        
        # 订阅无人机位姿
        self.create_subscription(
            Odometry,
            '/odom_world',
            self.odom_callback,
            10
        )
        
        # VLM 线程
        self.vlm_queue = queue.Queue()
        self.vlm_thread = threading.Thread(target=self.vlm_loop)
        self.vlm_thread.daemon = True
        self.vlm_thread.start()
        
        # 主循环定时器 (10Hz)
        self.create_timer(0.1, self.planning_loop)
        
        self.get_logger().info("="*50)
        self.get_logger().info("AirHunt 节点启动")
        self.get_logger().info(f"指令: {self.instruction}")
        self.get_logger().info(f"目标区域: ({TARGET_X}, {TARGET_Y})")
        self.get_logger().info(f"发布目标到: {GOAL_TOPIC}")
        self.get_logger().info("="*50)
    
    def odom_callback(self, msg):
        """接收无人机位姿"""
        self.current_pos[0] = msg.pose.pose.position.x
        self.current_pos[1] = msg.pose.pose.position.y
        self.current_pos[2] = msg.pose.pose.position.z
    
    def vlm_loop(self):
        """VLM 线程：慢速推理 (1Hz)"""
        while rclpy.ok():
            if self.found:
                time.sleep(1)
                continue
            
            self.step += 1
            
            # 调用 VLM 获取语义值
            value = self.vlm.infer(self.instruction, self.current_pos)
            self.get_logger().info(f"🔍 Step {self.step}: VLM 语义值 = {value:.3f}")
            
            # 更新价值地图
            self.value_map.update(self.current_pos[0], self.current_pos[1], value)
            
            # 检查是否找到目标
            if value > 0.85:
                self.get_logger().info(f"✅ 目标找到！位置: {self.current_pos}")
                self.found = True
                return
            
            # 获取下一个目标点
            target, best_val = self.value_map.get_best_target()
            self.get_logger().info(f"🎯 最佳目标: ({target[0]:.1f}, {target[1]:.1f}), 价值 = {best_val:.3f}")
            self.vlm_queue.put(target)
            
            time.sleep(1.0)
    
    def planning_loop(self):
        """规划主循环：发布目标点"""
        if self.found:
            return
        
        try:
            target = self.vlm_queue.get_nowait()
            self._publish_goal(target)
        except queue.Empty:
            pass
    
    def _publish_goal(self, target):
        """发布目标点给 Fast-Planner"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        
        pose = PoseStamped()
        pose.pose.position.x = float(target[0])
        pose.pose.position.y = float(target[1])
        pose.pose.position.z = 1.0
        pose.pose.orientation.w = 1.0
        
        path_msg.poses.append(pose)
        self.goal_pub.publish(path_msg)
        self.get_logger().info(f"📡 发布目标点: ({target[0]:.1f}, {target[1]:.1f})")


def main(args=None):
    rclpy.init(args=args)
    node = AirHuntNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
