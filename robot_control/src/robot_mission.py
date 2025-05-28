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


    def navigateMission(self):
        """
        導航任務
        """
        self.node.get_logger().info("[flow1]導航開啟")

    
    def run_find_3m(self):
        """
        這個函數會呼叫 find_3m.py 的功能
        """
        # 使用 subprocess 執行 find_3m.py
        self.node.get_logger().info("[flow1]找最佳導航點")
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
        
    def robot_find_fire(self):
        """
        這個函數會呼叫 找火源
        """
        self.node.get_logger().info("[flow1] 🔍 找火源")

    def nozzel_point(self):
        """
        這個函數會呼叫 nozzel控制
        """
        self.node.get_logger().info("[flow1] 🔫 噴頭點位")
        # tra_process = subprocess.Popen(["ros2", "run", "robot_control", "spray_trajectories.py"]) #求出噴頭角度

    def nozzel_move(self):
        """
        這個函數會呼叫 nozzel控制
        """
        self.node.get_logger().info("[flow1]💦噴頭移動")
        # nozzel_process = subprocess.Popen(["ros2", "run", "robot_control", "main_new.py"]) #噴頭控制
        # spray_process = subprocess.Popen(["ros2", "run", "robot_control", "open.py"]) #開啟抽水馬達io 

    def robot_goback(self):
        """
        這個函數會呼叫 返回原點
        """
        self.node.get_logger().info("[flow1] 🔙 返回原點")
        # goback_process = subprocess.Popen(["ros2", "run", "robot_control", "goback_node.py"]) #要改位置
