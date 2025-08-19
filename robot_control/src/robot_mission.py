    # ---------------------------------------------------------------------------- #
    #                                    Mission                                   #
    # ---------------------------------------------------------------------------- #
    #範例:
    # def templateMission(self):
    #     # 檢查先前模式是否為等待模式，定且設定目前模式為導航模式
    #     if self.mode != self.WAIT_MODE:
    #         self.stopMission()
    #         return False
    #     self.__setMode(self.TEMPLATE_MODE)
    #     # --------------------------------- variable --------------------------------- #
    #     # ----------------------------------- 開始任務 ----------------------------------- #
    #     if self.mode != self.TEMPLATE_MODE:
    #         self.node.get_logger().info("It's not in template mode")
    #         self.stopMission()
    #         return False
    #     # ----------------------------------- 結束任務 ----------------------------------- #
    #     self.stopMission()
    #     return True
import math
import rclpy
from rclpy.node import Node
import time
import yaml
import json
import os
import subprocess


class Mission:

    # define mode name
    WAIT_MODE = -1
    NAVIGATE_MODE = 0
    FIRE_MODE = 1
    SPRAY_MODE = 2
    GOHOME_MODE = 3

    # initial mode
    mode = WAIT_MODE
    def __init__(self, node):
        self.node = node
        pass



    def __setMode(self, mode):

        if mode not in [
            self.WAIT_MODE,
            self.NAVIGATE_MODE,
            self.FIRE_MODE,
            self.SPRAY_MODE,
            self.GOHOME_MODE
        ]:
            self.node.get_logger().error("not a valid mode")
            return False
        self.mode = mode

    def stopMission(self):
        self.__setMode(self.WAIT_MODE)
        self.controller.setZeroVelocity()

    
    def run_find_3m(self):
        """
        這個函數會呼叫 find_3m.py 的功能
        """
        # 使用 subprocess 執行 find_3m.py
        print("[flow1]找最佳導航點")
        json_path = "/home/robot/ivan_ws/src/robot_nav2/src/fire_target.json"
        process = subprocess.Popen(["ros2", "run", "robot_nav2", "fire_point_node.py"])

        timeout = 5  # 設定超時時間（秒）
        interval = 0.5
        waited = 0
        last_modified = os.path.getmtime(json_path) if os.path.exists(json_path) else 0
        while waited < timeout:
            if os.path.exists(json_path):
                new_time = os.path.getmtime(json_path)
                if new_time > last_modified:
                    self.get_logger().info("[flow1] ✅ fire_target.json 已更新")
                    with open(json_path, "r") as f:
                        pose = json.load(f)
                    process.terminate()
                    return pose
            time.sleep(interval)
            waited += interval

        self.get_logger().error("[flow1] ❌ 找點超時，沒有更新 JSON")
        process.terminate()
        return None
    
    def run_waypoint_node(self):
        """
        這個函數會呼叫 waypoint_node.py 的功能
        """
        # 使用 subprocess 執行 waypoint_node.py
        self.node.get_logger().info("[flow1]🚗導航到最佳導航點")
        process = subprocess.run(["python3", "/home/robot/ivan_ws/src/robot_nav2/src/waypoint_node.py"])
        imu_process = subprocess.Popen(["ros2", "launch", "realsense_camera", "imu_launch.py"])
        if process.returncode == 0:
            self.get_logger().info("[flow1] ✅ 導航成功")
            imu_process.terminate()
            return True
        else:
            self.get_logger().error("[flow1] ❌ 導航失敗")
            imu_process.terminate()
            return False
        
    def flow1(self):
        """
        滅火流程
        找點定位 waypoint --> 導航過去
        """
        print("[flow1] 🔥 滅火流程開始")
        pose = self.run_find_3m()
        if pose is None:
            print("[flow1] ❌ 找點失敗，流程結束")
            return False
        result = self.run_waypoint_node()
        if not result:
            print("[flow1] ❌ 導航失敗，流程結束")
            return False
        print("[flow1] ✅ 滅火流程結束")
    

if __name__ == "__main__":
    rclpy.init()
    node = Node("mission_node")
    mission = Mission(node)
    mission.flow1()
    rclpy.shutdown()

