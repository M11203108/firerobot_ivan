#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from thermal_msgs.msg import ThermalAlert
import math
import yaml
import cv2
import numpy as np
from tf_transformations import quaternion_from_euler
import time
import json

class FirePointNode(Node):
    def __init__(self):
        super().__init__('fire_point_node')

        yaml_path = "/home/robot/ivan_ws/src/robot_nav2/maps/map1.yaml"
        pgm_path = "/home/robot/ivan_ws/src/robot_nav2/maps/map1.pgm"

        # 讀取地圖
        self.map_img, self.resolution, self.origin = self.load_map(yaml_path, pgm_path)

        # 機器人初始位置
        self.robot_pos = (0.0, 0.0)
        self.best_pose = None


        #訂閱火源
        self.create_subscription(
            ThermalAlert,
            "/thermal_alert",
            self.fire_callback,
            rclpy.qos.qos_profile_sensor_data,
        )

        time.sleep(0.5)
    
    def load_map(self, yaml_path, pgm_path):
        with open(yaml_path, 'r') as f:
            map_info = yaml.safe_load(f)
        resolution = map_info['resolution']
        origin = map_info['origin']
        img = cv2.imread(pgm_path, cv2.IMREAD_UNCHANGED)
        return img, resolution, origin
    
    def world_to_map(self, x, y):
        mx = int((x - self.origin[0]) / self.resolution)
        my = self.map_img.shape[0] - int((y - self.origin[1]) / self.resolution)
        return mx, my
    
    def generate_circle_points(self, center_xy, radius, num_points):
        cx, cy = center_xy
        points = []
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)
            points.append((x, y))
        return points
    
    def is_area_navigable(self, x, y, radius):
        mx_center, my_center = self.world_to_map(x, y)
        pixel_radius = int(radius / self.resolution)
        h, w = self.map_img.shape

        for dx in range(-pixel_radius, pixel_radius + 1):
            for dy in range(-pixel_radius, pixel_radius + 1):
                if dx**2 + dy**2 <= pixel_radius**2:
                    mx = mx_center + dx
                    my = my_center + dy
                    if 0 <= mx < w and 0 <= my < h:
                        if self.map_img[my, mx] < 254:  # ✅ 只允許完全白色
                            return False
                    else:
                        return False  # 出界也視為不可行
        return True
    
    def find_clear_line(self, p1, p2, line_width=1.0):
        mx1, my1 = self.world_to_map(p1[0], p1[1])
        mx2, my2 = self.world_to_map(p2[0], p2[1])

        mask = np.zeros_like(self.map_img, dtype=np.uint8)

        # 線寬轉為像素
        line_thickness = max(1, int((line_width / self.resolution)))

        # 畫白色線
        cv2.line(mask, (mx1, my1), (mx2, my2), 255, thickness=line_thickness)

        # 找出線上所有的像素 (非0)
        ys, xs = np.where(mask > 0)

        # 檢查線上是否有障礙物（<254）
        for (x, y) in zip(xs, ys):
            if self.map_img[y, x] < 254:
                return False
        return True
    
    def find_nearest_navigable_point(self, points, robot_pos):
        valid_points = []
        for point in points:
            if self.is_area_navigable(point[0], point[1], radius=1.0):
                dist = math.hypot(point[0] - robot_pos[0], point[1] - robot_pos[1])
                valid_points.append((dist,point))
        if valid_points:
            valid_points.sort()
            return valid_points[0][1]
        else:
            return None
        
    def fire_callback(self, msg):
        


        hotspot_world = (msg.x, msg.y)
        self.get_logger().info(f'🔥 收到火源點: {hotspot_world}')

        temp = msg.temperature

        # 產生圓周點
        circle_points = self.generate_circle_points(hotspot_world, radius=3.5, num_points=72)

        valid_points = []
        invalid_points = []
        for point in circle_points:
            if self.is_area_navigable(point[0], point[1], radius=1.0):
                if self.find_clear_line(point, hotspot_world, line_width=1.0):
                    valid_points.append(point)
                else:
                    invalid_points.append(point)
            else:
                invalid_points.append(point)

        # 找最近點
        best_point = self.find_nearest_navigable_point(valid_points, self.robot_pos)

        if best_point:
            self.get_logger().info(f"✅ 找到的最佳導航點: {best_point}")

            # 計算朝向火源的 orientation
            dx = hotspot_world[0] - best_point[0]
            dy = hotspot_world[1] - best_point[1]
            yaw = math.atan2(dy, dx)
            qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)

            # 存檔資料
            save_data = {
                "x": best_point[0],
                "y": best_point[1],
                "z": 0.0,
                "qx": qx,
                "qy": qy,
                "qz": qz,
                "qw": qw
            }

            with open("/home/robot/ivan_ws/src/robot_nav2/src/fire_target.json", "w") as f:
                json.dump(save_data, f)
            self.get_logger().info("💾 已將導航點儲存到 /tmp/fire_target.json")

            self.best_pose = save_data
        else:
            self.get_logger().info("❌ 沒有找到可導航的點")

def main(args=None):
    rclpy.init(args=args)
    fire_point_node = FirePointNode()
    rclpy.spin(fire_point_node)
    fire_point_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        
    