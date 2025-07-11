#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import json
import sys
import threading

# 加在開頭 main() 前面
cancel_flag = False

def listen_for_cancel():
    global cancel_flag
    print("👉 請輸入 'c' 並按 Enter 可取消導航")
    while True:
        user_input = input()
        if user_input.lower() == 'c':
            cancel_flag = True
            break

def main():
    rclpy.init()

    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    print('✅ Nav2 已啟動，等待目標點...')

    try:
        with open("/home/robot/ivan_ws/src/robot_nav2/src/fire_target.json", "r") as f:
            data = json.load(f)
    except FileNotFoundError:
        print("❌ 找不到 fire_target.json")
        return

    print(f"📝 讀到的 JSON 內容: {json.dumps(data, indent=2)}")

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = data["x"]
    goal_pose.pose.position.y = data["y"]
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = data["qx"]
    goal_pose.pose.orientation.y = data["qy"]
    goal_pose.pose.orientation.z = data["qz"]
    goal_pose.pose.orientation.w = data["qw"]

    print(f'📍 導航到: ({data["x"]:.2f}, {data["y"]:.2f}), 開始導航...')

    navigator.goToPose(goal_pose)

    cancel_thread = threading.Thread(target=listen_for_cancel, daemon=True)
    cancel_thread.start()

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f'🚗 距離剩餘: {feedback.distance_remaining:.2f} meters')
        if cancel_flag:
            print('🚫 導航被取消，正在停止...')
            navigator.cancelTask()
            break

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('🎉 導航成功！')
        sys.exit(0) # 0 成功
    elif result == TaskResult.CANCELED:
        print('⚠️ 導航被取消')
    elif result == TaskResult.FAILED:
        print('❌ 導航失敗')
        sys.exit(1) # 1 失敗
    else:
        print('❓ 導航回傳未知狀態')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
