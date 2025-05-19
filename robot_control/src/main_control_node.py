#!/usr/bin/python3
import time
import os
import rclpy
import json
import subprocess
from rclpy.node import Node
import threading
from thermal_msgs.msg import ThermalAlert
from robot_mission import Mission

class MainControlNode(Node):

    # flow mode
    STOP_FLOW = 0
    FLOW1_FLOW = 1
    TEST_FLOW = 2

    # incoming message
    thermal_alert_msg = ThermalAlert()

    def __init__(self):

        # 創建控制實例
        self.mission = Mission(self)

        self.create_subscription(
            ThermalAlert,
            "/thermal_alert",
            self.thermalAlertCallback,
            rclpy.qos.qos_profile_sensor_data,
        )

        self.flow_thread = threading.Thread()  # 建立空執行緒
        self.flow_mode = self.STOP_FLOW  # 暫時使用的流程模式
        self.create_timer(1, self.mainDetectCallback)  # 建立主循環的定時器

        # 啟動導航
        

    # ---------------------------------------------------------------------------- #
    #                                   main loop                                  #
    # ---------------------------------------------------------------------------- #

    def mainDetectCallback(self):
        """
        此為主要的循環
        持續的偵測流程模式，並切換流程
        """

        def flowSwitch(target, target_name):
            """
            detectCallback()的子函數，用來切換流程，並確保只有一個流程在執行
            """
            if not self.flow_thread.is_alive():
                self.flow_thread = threading.Thread(target=target, name=target_name)
                self.flow_thread.start()
                return
            if self.flow_thread.is_alive() and self.flow_thread.name == target_name:
                self.get_logger().info(f"{target_name} is running")
                return
            if self.flow_thread.is_alive() and self.flow_thread.name != target_name:
                self.mission.stopMission()
                self.flow_thread = threading.Thread(target=target, name=target_name)
                self.flow_thread.start()
                return
        # 判斷使用哪個流程
        if self.flow_mode == self.STOP_FLOW:
            self.mission.stopMission()
            self.get_logger().info("stop flow")
            return
        elif self.flow_mode == self.FLOW1_FLOW:
            flowSwitch(self.flow1, "flow1")
            return
        elif self.flow_mode == self.TEST_FLOW:
            flowSwitch(self.testFlow, "testFlow")
            return
        
    # ---------------------------------------------------------------------------- #
    #                                  detect rule                                 #
    # ---------------------------------------------------------------------------- #

    def thermalAlertCallback(self, msg):
        """
        火源警報的callback函數
        """
        # self.get_logger().info(f"thermal alert: {msg}")
        # 如果溫度小於60度，則不執行
        if msg.temperature < 60:
            return
        self.thermal_alert_msg = msg
        self.flow_mode = self.FLOW1_FLOW

    # def testFlow(self):
    #     if not self.mission.loadingExtinguisher():
    #         self.get_logger().info("load extinguish fail")
    #         self.flow_mode = self.STOP_FLOW
    #         return
    #     self.flow_mode = self.STOP_FLOW

    def flow1(self):
        """
        滅火流程
        找點定位waypoint -->開啟熱像儀傳熱點訊息--> 量測距離傳給噴頭設定角度 --> 邊噴邊動噴頭-->返回原點
        """
        if not self.mission.navigateMission():
            pass
        if not self.mission.sprayMission():
            pass
    
    def run_find_3m(self):
        """
        這個函數會呼叫 find_3m.py 的功能
        """
        # 使用 subprocess 執行 find_3m.py
        self.get_logger().info("[flow1]找最佳導航點")
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
        self.get_logger().info("[flow1]🚗導航到最佳導航點")
        process = subprocess.run(["python3", "/home/robot/ivan_ws/src/robot_nav2/src/waypoint_node.py"])
        if process.returncode == 0:
            self.get_logger().info("[flow1] ✅ 導航成功")
            return True
        else:
            self.get_logger().error("[flow1] ❌ 導航失敗")
            return False


def main():
    if not rclpy.ok():
        rclpy.init()
    control_node = MainControlNode()
    control_node.flow_mode = control_node.STOP_FLOW
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()