#!/usr/bin/env python3
"""
AirHunt 节点 - 探索模式（强制扩大探索范围）
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

# ========== 配置 ==========
GOAL_TOPIC = "/waypoint_generator/waypoints"
MAP_WIDTH = 30.0
MAP_HEIGHT = 30.0
RESOLUTION = 0.8  # 降低分辨率，扩大探索范围

# VLM API 配置
VLM_API_KEY = "sk-kemzagtjgimzlfcadjzowhxknszkiflojjkrdklkyibtpvux"
VLM_BASE_URL = "https://api.siliconflow.cn/v1"
VLM_MODEL = "Pro/Qwen/Qwen2-VL-7B-Instruct"


# ========== 价值地图（带探索激励）==========
class ValueMap3D:
    def __init__(self, width, height, resolution):
        self.width = width
        self.height = height
        self.res = resolution
        nx = int(width / resolution) + 1
        ny = int(height / resolution) + 1
        self.grid = np.zeros((nx, ny))
        self.visited = np.zeros((nx, ny))  # 访问次数计数
        self.conf = np.zeros((nx, ny))
    
    def xy_to_idx(self, x, y):
        ix = int(x / self.res)
        iy = int(y / self.res)
        ix = max(0, min(ix, self.grid.shape[0] - 1))
        iy = max(0, min(iy, self.grid.shape[1] - 1))
        return ix, iy
    
    def update(self, x, y, value, confidence=0.7):
        ix, iy = self.xy_to_idx(x, y)
        self.visited[ix, iy] += 1
        old_v = self.grid[ix, iy]
        old_c = self.conf[ix, iy]
        new_v = (old_c * old_v + confidence * value) / (old_c + confidence + 1e-6)
        new_c = min(1.0, old_c + confidence)
        self.grid[ix, iy] = new_v
        self.conf[ix, iy] = new_c
    
    def get_best_target(self):
        # 探索激励：未访问区域 + 高语义值
        exploration_bonus = 0.3 * np.exp(-self.visited / 2.0)
        score = self.grid + exploration_bonus
        idx = np.unravel_index(np.argmax(score), score.shape)
        x = idx[0] * self.res
        y = idx[1] * self.res
        # 边界限制（确保在地图内）
        x = max(-12.0, min(12.0, x))
        y = max(-10.0, min(10.0, y))
        return np.array([x, y, 1.5]), self.grid[idx]


# ========== 真实 VLM 客户端 ==========
class RealVLMText:
    def __init__(self, api_key, base_url, model):
        self.api_key = api_key
        self.base_url = base_url
        self.model = model
        self.session = requests.Session()
    
    def infer(self, instruction, scene_description):
        prompt = f"""Rate the probability (0-1) that this scene contains: {instruction}. Scene: {scene_description} Output ONLY a number."""
        headers = {"Authorization": f"Bearer {self.api_key}", "Content-Type": "application/json"}
        payload = {"model": self.model, "messages": [{"role": "user", "content": prompt}], "max_tokens": 10}
        try:
            response = self.session.post(f"{self.base_url}/chat/completions", headers=headers, json=payload, timeout=10)
            if response.status_code == 200:
                text = response.json()["choices"][0]["message"]["content"].strip()
                import re
                match = re.search(r"0?\.\d+|\b[01]\b", text)
                if match:
                    return float(match.group())
            return 0.5
        except Exception as e:
            print(f"VLM error: {e}")
            return 0.5


# ========== 场景描述生成器 ==========
class SceneDescriptionGenerator:
    def generate(self, pos):
        x, y = pos[0], pos[1]
        dist_to_center = math.sqrt(x**2 + y**2)
        if dist_to_center < 2.0:
            return "A green trash bin on the roadside, clearly visible."
        elif dist_to_center < 4.0:
            return "A roadside area with a green trash bin in the distance."
        elif dist_to_center < 6.0:
            return "A grassy area with a path and some trees. A green trash bin is visible."
        else:
            return "An open field with scattered trees."


# ========== AirHunt 节点 ==========
class AirHuntNode(Node):
    def __init__(self):
        super().__init__('airhunt_node')
        
        self.declare_parameter('instruction', 'find a green trash bin')
        self.instruction = self.get_parameter('instruction').value
        
        self.vlm = RealVLMText(VLM_API_KEY, VLM_BASE_URL, VLM_MODEL)
        self.scene_gen = SceneDescriptionGenerator()
        self.value_map = ValueMap3D(MAP_WIDTH, MAP_HEIGHT, RESOLUTION)
        self.current_pos = [0.0, 0.0, 1.5]
        self.found = False
        self.step = 0
        
        self.goal_pub = self.create_publisher(Path, GOAL_TOPIC, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.vlm_queue = queue.Queue()
        self.vlm_thread = threading.Thread(target=self.vlm_loop)
        self.vlm_thread.daemon = True
        self.vlm_thread.start()
        
        self.create_timer(0.1, self.planning_loop)
        
        self.get_logger().info("="*50)
        self.get_logger().info("AirHunt 探索模式启动")
        self.get_logger().info(f"指令: {self.instruction}")
        self.get_logger().info("="*50)
    
    def odom_callback(self, msg):
        self.current_pos[0] = msg.pose.position.x
        self.current_pos[1] = msg.pose.position.y
    
    def vlm_loop(self):
        while rclpy.ok() and not self.found:
            self.step += 1
            scene = self.scene_gen.generate(self.current_pos)
            self.get_logger().info(f"🔍 Step {self.step}: 位置 ({self.current_pos[0]:.1f}, {self.current_pos[1]:.1f})")
            
            value = self.vlm.infer(self.instruction, scene)
            self.get_logger().info(f"   VLM = {value:.3f}")
            
            self.value_map.update(self.current_pos[0], self.current_pos[1], value)
            
            target, _ = self.value_map.get_best_target()
            self.get_logger().info(f"🎯 目标点: ({target[0]:.1f}, {target[1]:.1f})")
            self.vlm_queue.put(target)
            
            time.sleep(2.5)
    
    def planning_loop(self):
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
        pose.pose.position.z = 1.5
        pose.pose.orientation.w = 1.0
        path_msg.poses.append(pose)
        self.goal_pub.publish(path_msg)
        self.get_logger().info(f"📡 发布: ({target[0]:.1f}, {target[1]:.1f})")


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
