#!/usr/bin/env python3
import rclpy, time
import os, json
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from pathlib import Path
from launch import LaunchService
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from fire_point_node import FirePointNode
from waypoint_node import WaypointNavigator 
from waypoint_back import BackWaypointNavigator
from thermal_msgs.msg import ThermalAlert
import threading

TEMPERATURE_THRESHOLD = 50.0          # °C
CHECK_INTERVAL       = 1.0 

class SimpleFireMission(Node):
    def __init__(self):
        super().__init__("simple_fire_mission")
        self.fire_done = threading.Event()
        self.latest_temp = None

        self.create_subscription(
            ThermalAlert,
            "/thermal_alert",
            self.temp_callback,
            rclpy.qos.qos_profile_sensor_data,
        )

    def temp_callback(self, msg): 
        self.latest_temp = msg.temperature

    def wait_until_hot(self):
        self.get_logger().info(f"🕒 Stand-by，等待溫度 ≥ {TEMPERATURE_THRESHOLD} °C …")
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.latest_temp is not None and \
            self.latest_temp >= TEMPERATURE_THRESHOLD:
                self.get_logger().info(
                    f"🔥 溫度達標 {self.latest_temp:.1f} °C，開始任務")
                return True

            time.sleep(CHECK_INTERVAL)
        return False

    def run_find_3m(self):
        fire_node = FirePointNode()                   # 1. 建節點
        exe = SingleThreadedExecutor()
        exe.add_node(fire_node)

        exe.spin_once(timeout_sec=1.0)                # 2. 只跑一次

        exe.remove_node(fire_node)                    # 3. 收掉
        fire_node.destroy_node()

        # 4. 直接讀檔結果
        json_path = "/home/robot/ivan_ws/src/robot_nav2/src/fire_target.json"
        if os.path.exists(json_path):
            with open(json_path, "r") as f:
                pose = json.load(f)
            self.get_logger().info(f"[flow1] ✅ 讀到 pose: {pose}")
            return pose

        self.get_logger().error("[flow1] ❌ 沒有 fire_target.json")
        return None
    
    def run_waypoint(self):
        self.get_logger().info("[flow1]🚗導航到最佳導航點")
        
        nav_node = WaypointNavigator()         
        ok = nav_node.navigate()       
        nav_node.destroy_node()                 

        if ok:
            self.get_logger().info("[flow1] ✅ 導航成功")
        else:
            self.get_logger().error("[flow1] ❌ 導航失敗")
        return ok
    
    def fire_fight(self):
        self.get_logger().info("[flow1]🔥 開始滅火")

        fire_fight_launch_file = Path("/home/robot/newjasmine_ws/src/rs485_fire/launch/test_wrapper_launch.py")
        
        self.ls_fire = LaunchService()
        launch_action = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(fire_fight_launch_file)))
        self.ls_fire.include_launch_description(launch_action)
        
        self.get_logger().info("💦 已啟動滅火 launch")
        self.ls_fire.run()
        self.get_logger().info("🔥 滅火流程完成")
        return True
    
    def back_to_base(self):
        self.get_logger().info("[flow1]🚗導航回原點")
        back_node = BackWaypointNavigator()
        ok = back_node.navigate()
        back_node.destroy_node()

        if ok:
            self.get_logger().info("[flow1] ✅ 回原點成功")
        else:
            self.get_logger().error("[flow1] ❌ 回原點失敗")
        return ok



    


def main(args=None):
    rclpy.init(args=args)
    mission = SimpleFireMission()

    if not mission.wait_until_hot():
        rclpy.shutdown()
        return

    pose = mission.run_find_3m()
    if pose is None:
        mission.get_logger().error("❌ 找點失敗，結束")
        rclpy.shutdown();  return

    print(f"🚗 導航到點: {pose}")
    if not mission.run_waypoint():
        mission.get_logger().error("❌ 導航失敗，結束")
        rclpy.shutdown();  return
    
    if not mission.fire_fight():
        mission.get_logger().error("❌ 滅火 launch 啟動失敗")
        rclpy.shutdown();  return
    
    print("🔥 滅火流程完成，準備回原點")

    if mission.back_to_base():
        mission.get_logger().info("🎉 全流程完成")
        rclpy.shutdown();  return
    else:
        mission.get_logger().error("❌ 回原點失敗")
        

    rclpy.shutdown()

if __name__ == "__main__":
    main()