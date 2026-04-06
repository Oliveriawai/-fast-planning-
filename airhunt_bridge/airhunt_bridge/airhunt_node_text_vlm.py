#!/usr/bin/env python3
"""
AirHunt 节点 - 接入真实 VLM（文字描述模式）
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np
import threading
import queue
import time
import math
import requests
import json

# ========== 配置 ==========
GOAL_TOPIC = "/waypoint_generator/waypoints"
MAP_WIDTH = 30.0
MAP_HEIGHT = 30.0
RESOLUTION = 0.5

# VLM API 配置
VLM_API_KEY = "sk-kemzagtjgimzlfcadjzowhxknszkiflojjkrdklkyibtpvux"
VLM_BASE_URL = "https://api.siliconflow.cn/v1"
VLM_MODEL = "Pro/Qwen/Qwen2-VL-7B-Instruct"


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
        # 边界限制（地图范围 X:-15~15, Y:-12~12）
        x = max(-12.0, min(12.0, x))
        y = max(-10.0, min(10.0, y))
        return np.array([x, y, 1.5]), self.grid[idx]


# ========== 真实 VLM 客户端（文字模式）==========
class RealVLMText:
    def __init__(self, api_key, base_url, model):
        self.api_key = api_key
        self.base_url = base_url
        self.model = model
        self.session = requests.Session()
    
    def infer(self, instruction, scene_description):
        prompt = f"""You are a drone navigation assistant. Based on the scene description, rate how likely it is that the target is present.

Scene description: {scene_description}
Instruction: {instruction}

Output ONLY a number between 0 and 1, where 0 means definitely not present, 1 means definitely present. Example: 0.85
"""
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json"
        }
        
        payload = {
            "model": self.model,
            "messages": [
                {"role": "user", "content": prompt}
            ],
            "max_tokens": 10,
            "temperature": 0.0
        }
        
        try:
            response = self.session.post(
                f"{self.base_url}/chat/completions",
                headers=headers,
                json=payload,
                timeout=10
            )
            
            if response.status_code == 200:
                result = response.json()
                text = result["choices"][0]["message"]["content"].strip()
                import re
                match = re.search(r"0?\.\d+|\b[01]\b", text)
                if match:
                    return float(match.group())
            return 0.5
        except Exception as e:
            print(f"VLM 调用失败: {e}")
            return 0.5


# ========== 场景描述生成器 ==========
class SceneDescriptionGenerator:
    """根据无人机位置生成场景描述"""
    def __init__(self, target_x=12.0, target_y=12.0):
        self.target_x = target_x
        self.target_y = target_y
    
    def generate(self, pos):
        x, y = pos[0], pos[1]
        dist_to_target = math.sqrt((x - self.target_x)**2 + (y - self.target_y)**2)
        
        if dist_to_target < 2.0:
            return "A green trash bin on the roadside, clearly visible. The bin is plastic, green, and placed next to a tree."
        elif dist_to_target < 5.0:
            return "A roadside area with a green trash bin visible in the distance. The bin is about 5 meters away, near a tree."
        elif dist_to_target < 10.0:
            return "A grassy area with a path and some trees. You can see a green trash bin in the distance."
        elif x < 5 or y < 5:
            return "A paved road with buildings on both sides. No trash bin visible."
        else:
            return "An open field with scattered trees and bushes. The area looks natural and open."


# ========== AirHunt ROS2 节点 ==========
class AirHuntNode(Node):
    def __init__(self):
        super().__init__('airhunt_node')
        
        # 参数
        self.declare_parameter('instruction', 'find the green trash bin on the roadside')
        self.instruction = self.get_parameter('instruction').value
        
        # 初始化
        self.vlm = RealVLMText(VLM_API_KEY, VLM_BASE_URL, VLM_MODEL)
        self.scene_generator = SceneDescriptionGenerator(target_x=12.0, target_y=12.0)
        self.value_map = ValueMap3D(MAP_WIDTH, MAP_HEIGHT, RESOLUTION)
        self.current_pos = [0.0, 0.0, 1.5]
        self.found = False
        self.step = 0
        
        # 发布器
        self.goal_pub = self.create_publisher(Path, GOAL_TOPIC, 10)
        
        # 订阅无人机位姿
        self.create_subscription(Odometry, '/odom_world', self.odom_callback, 10)
        
        # 队列
        self.vlm_queue = queue.Queue()
        
        # 启动 VLM 线程
        self.vlm_thread = threading.Thread(target=self.vlm_loop)
        self.vlm_thread.daemon = True
        self.vlm_thread.start()
        
        # 主循环定时器 (10Hz)
        self.create_timer(0.1, self.planning_loop)
        
        self.get_logger().info("="*50)
        self.get_logger().info("AirHunt 节点启动 (真实 VLM - 文字模式)")
        self.get_logger().info(f"指令: {self.instruction}")
        self.get_logger().info(f"VLM 模型: {VLM_MODEL}")
        self.get_logger().info(f"目标区域: (12.0, 12.0)")
        self.get_logger().info(f"发布目标到: {GOAL_TOPIC}")
        self.get_logger().info("="*50)
    
    def odom_callback(self, msg):
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
            
            # 生成场景描述
            scene_desc = self.scene_generator.generate(self.current_pos)
            self.get_logger().info(f"🔍 Step {self.step}: 当前位置 ({self.current_pos[0]:.1f}, {self.current_pos[1]:.1f})")
            self.get_logger().info(f"   场景描述: {scene_desc[:50]}...")
            
            # 调用真实 VLM
            self.get_logger().info(f"   调用 VLM...")
            value = self.vlm.infer(self.instruction, scene_desc)
            self.get_logger().info(f"   VLM 语义值 = {value:.3f}")
            
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
            
            time.sleep(2.0)
    
    def planning_loop(self):
        """规划主循环：发布目标点 (10Hz)"""
        if self.found:
            return
        
        try:
            target = self.vlm_queue.get_nowait()
            self._publish_goal(target)
        except queue.Empty:
            pass
    
    def _publish_goal(self, target):
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
