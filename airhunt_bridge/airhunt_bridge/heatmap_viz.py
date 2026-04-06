#!/usr/bin/env python3
"""
价值地图热力图发布器 - 在 Rviz 中显示
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np

class HeatworldViz(Node):
    def __init__(self):
        super().__init__('heatworld_viz')
        self.pub = self.create_publisher(MarkerArray, '/airhunt/heatworld', 10)
        self.timer = self.create_timer(1.0, self.publish_heatworld)
        
        # 模拟价值地图中心
        self.center_x = 0.0
        self.center_y = 0.0
        
        self.get_logger().info("热力图可视化节点启动")
    
    def publish_heatworld(self):
        marker_array = MarkerArray()
        marker_id = 0
        
        # 显示范围 -10 到 10，步长 0.8
        for x in np.arange(-10, 10, 0.8):
            for y in np.arange(-10, 10, 0.8):
                # 计算价值（距离中心越近价值越高）
                dist = np.sqrt((x - self.center_x)**2 + (y - self.center_y)**2)
                value = max(0.0, min(1.0, 1.0 - dist / 12.0))
                
                if value > 0.05:
                    marker = Marker()
                    marker.header.frame_id = "world"
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = "heatworld"
                    marker.id = marker_id
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD
                    marker.pose.position.x = float(x)
                    marker.pose.position.y = float(y)
                    marker.pose.position.z = 0.3
                    marker.scale.x = 0.6
                    marker.scale.y = 0.6
                    marker.scale.z = 0.1
                    
                    # 颜色：红色 = 高价值，蓝色 = 低价值
                    # 使用 float 类型，确保是 0-1 范围
                    marker.color.r = float(min(1.0, value * 1.5))
                    marker.color.g = 0.2
                    marker.color.b = float(max(0.0, 1.0 - value))
                    marker.color.a = 0.6
                    
                    marker_array.markers.append(marker)
                    marker_id += 1
        
        self.pub.publish(marker_array)
        self.get_logger().debug(f"发布了 {marker_id} 个热力图方块")


def main(args=None):
    rclpy.init(args=args)
    node = HeatworldViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
