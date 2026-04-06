#!/usr/bin/env python3
"""
AirHunt 节点 - 带调试信息
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header
import numpy as np
import threading
import queue
import time
import math
import requests

GOAL_TOPIC = "/waypoint_generator/waypoints"

VLM_API_KEY = "sk-kemzagtjgimzlfcadjzowhxknszkiflojjkrdklkyibtpvux"
VLM_BASE_URL = "https://api.siliconflow.cn/v1"
VLM_MODEL = "Pro/Qwen/Qwen2-VL-7B-Instruct"


class RealVLMText:
    def __init__(self, api_key, base_url, model):
        self.api_key = api_key
        self.base_url = base_url
        self.model = model
        self.session = requests.Session()
    
    def infer(self, instruction, scene_description):
        prompt = f"Rate the probability (0-1) that this scene contains: {instruction}. Scene: {scene_description} Output ONLY a number."
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


class SceneDescriptionGenerator:
    def generate(self, pos):
        x, y = pos[0], pos[1]
        dist = math.sqrt((x - 12)**2 + (y - 12)**2)
        if dist < 2.0:
            return "A green trash bin on the roadside, clearly visible."
        elif dist < 5.0:
            return "A roadside area with a green trash bin in the distance."
        elif dist < 10.0:
            return "A grassy area with a path. A green trash bin is in the distance."
        else:
            return "An open field with scattered trees."


class ValueMap3D:
    def __init__(self, width, height, resolution):
        self.width = width
        self.height = height
        self.res = resolution
        nx = int(width / resolution) + 1
        ny = int(height / resolution) + 1
        self.grid = np.zeros((nx, ny))
        self.visited = np.zeros((nx, ny))
    
    def xy_to_idx(self, x, y):
        ix = int(x / self.res)
        iy = int(y / self.res)
        ix = max(0, min(ix, self.grid.shape[0] - 1))
        iy = max(0, min(iy, self.grid.shape[1] - 1))
        return ix, iy
    
    def update(self, x, y, value):
        ix, iy = self.xy_to_idx(x, y)
        self.visited[ix, iy] += 1
        self.grid[ix, iy] = self.grid[ix, iy] * 0.7 + value * 0.3
    
    def get_best_target(self):
        exploration_bonus = 0.3 * np.exp(-self.visited / 2.0)
        score = self.grid + exploration_bonus
        idx = np.unravel_index(np.argmax(score), score.shape)
        x = idx[0] * self.res
        y = idx[1] * self.res
        x = max(-12.0, min(12.0, x))
        y = max(-10.0, min(10.0, y))
        return np.array([x, y, 1.5]), self.grid[idx]


class AirHuntNode(Node):
    def __init__(self):
        super().__init__('airhunt_node')
        
        self.declare_parameter('instruction', 'find a green trash bin')
        self.instruction = self.get_parameter('instruction').value
        
        self.vlm = RealVLMText(VLM_API_KEY, VLM_BASE_URL, VLM_MODEL)
        self.scene_gen = SceneDescriptionGenerator()
        self.value_map = ValueMap3D(30.0, 30.0, 0.8)
        self.current_pos = [0.0, 0.0, 1.5]
        self.found = False
        self.step = 0
        
        self.goal_pub = self.create_publisher(Path, GOAL_TOPIC, 10)
        
        # 订阅 /pose 话题
        self.pose_sub = self.create_subscription(
            Pose,
            '/pose',
            self.pose_callback,
            10
        )
        
        self.get_logger().info("等待 /pose 消息...")
        
        self.vlm_queue = queue.Queue()
        self.vlm_thread = threading.Thread(target=self.vlm_loop)
        self.vlm_thread.daemon = True
        self.vlm_thread.start()
        
        self.create_timer(0.1, self.planning_loop)
        
        self.get_logger().info("="*50)
        self.get_logger().info("AirHunt 节点启动 (调试版)")
        self.get_logger().info(f"指令: {self.instruction}")
        self.get_logger().info("="*50)
    
    def pose_callback(self, msg):
        """接收 /pose 消息"""
        self.current_pos[0] = msg.position.x
        self.current_pos[1] = msg.position.y
        self.current_pos[2] = msg.position.z
        self.get_logger().info(f"📍 位置更新: ({self.current_pos[0]:.1f}, {self.current_pos[1]:.1f})")
    
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
