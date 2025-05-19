
import time
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import json

def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()
    with open('/home/robot/ivan_ws/src/robot_nav2/maps/fire_point.json', 'r') as f:
        waypoints = json.load(f)

    goal_poses = []
    for wp in waypoints:
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = wp["x"]
        goal_pose.pose.position.y = wp["y"]
        goal_pose.pose.position.z = wp["z"]
        goal_pose.pose.orientation.x = wp["qx"]
        goal_pose.pose.orientation.y = wp["qy"]
        goal_pose.pose.orientation.z = wp["qz"]
        goal_pose.pose.orientation.w = wp["qw"]
        goal_poses.append(goal_pose)

    while rclpy.ok():
        for goal_pose in goal_poses:
            navigator.goThroughPoses([goal_pose])

            while not navigator.isTaskComplete():
                feedback = navigator.getFeedback()
                if feedback:
                    print('Distance remaining: {:.2f} meters'.format(feedback.distance_remaining))

            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Goal succeeded!')
            elif result == TaskResult.CANCELED:
                print('Goal was canceled!')
            elif result == TaskResult.FAILED:
                print('Goal failed!')
            else:
                print('Goal has an invalid return status!')
                
            time.sleep(3)

    navigator.lifecycleShutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()