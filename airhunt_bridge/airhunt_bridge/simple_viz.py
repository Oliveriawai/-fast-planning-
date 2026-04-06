#!/usr/bin/env python3
"""
简单可视化 - 直接打印价值地图到终端
"""

import rclpy
from rclpy.node import Node
import numpy as np

class SimpleViz(Node):
    def __init__(self):
        super().__init__('simple_viz')
        # 模拟价值地图
        self.grid = np.zeros((30, 30))
        self.center_x = 0
        self.center_y = 0
        self.timer = self.create_timer(2.0, self.print_map)
        
    def print_map(self):
        print("\n" + "="*50)
        print("价值地图热力图（数字越大价值越高）")
        print("="*50)
        
        # 打印一个 20x20 的区域
        for i in range(-10, 10):
            row = []
            for j in range(-10, 10):
                dist = np.sqrt((i - self.center_x)**2 + (j - self.center_y)**2)
                value = max(0, 1 - dist / 15.0)
                if value > 0.5:
                    row.append("█")
                elif value > 0.3:
                    row.append("▓")
                elif value > 0.1:
                    row.append("▒")
                else:
                    row.append("░")
            print("".join(row))
        print("中心(0,0)是目标区域")
        print("="*50 + "\n")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
