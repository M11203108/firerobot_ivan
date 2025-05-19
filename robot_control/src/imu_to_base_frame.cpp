#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class ImuAligner : public rclcpp::Node
{
public:
  ImuAligner() : Node("imu_aligner")
  {
    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 10,
      std::bind(&ImuAligner::imuCallback, this, std::placeholders::_1));

    pub_imu_ = create_publisher<sensor_msgs::msg::Imu>(
      "/imu/data_aligned", 10);

    // 將IMU原始座標系轉換成ROS標準座標系的旋轉矩陣
    mat_.setValue(
        1, 0, 0,   // X_new = X_old (IMU 前方對應 ROS 前方)
        0, -1, 0,  // Y_new = -Y_old (IMU 右邊對應 ROS 左邊)
        0, 0, 1    // Z_new = Z_old (IMU 上方對應 ROS 上方)
    );


    // 設定 IMU 偏移量 (假設 IMU 相對於 base_link 的位置)
    imu_offset_.setValue(
      0.4754, 0.13298, 0.9801
      );  // 修改為你的 IMU 真實位置 (x, y, z)
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
      sensor_msgs::msg::Imu aligned_msg = *msg;

      // 將 IMU 原始 orientation 提取出來
      tf2::Quaternion q_orig;
      tf2::fromMsg(msg->orientation, q_orig);

      // 將 quaternion 轉成矩陣形式 (3x3)
      tf2::Matrix3x3 orig_mat(q_orig);

      // 用你自訂的矩陣 `mat_` 將 IMU 資料轉換到 ROS 座標系
      tf2::Matrix3x3 transformed_mat = mat_ * orig_mat;

      // 將轉換後的矩陣轉換回 quaternion
      tf2::Quaternion q_aligned;
      transformed_mat.getRotation(q_aligned);
      aligned_msg.orientation = tf2::toMsg(q_aligned);

      // 轉換 Angular Velocity
      tf2::Vector3 angular(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
      angular = mat_ * angular;
      aligned_msg.angular_velocity.x = angular.x();
      aligned_msg.angular_velocity.y = angular.y();
      aligned_msg.angular_velocity.z = angular.z();

      // 轉換 Linear Acceleration
      tf2::Vector3 accel(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
      accel = mat_ * accel;

      // 補償 IMU 偏移量對加速度的影響 (考慮角速度影響)
      tf2::Vector3 angular_velocity(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
      tf2::Vector3 acc_offset = angular_velocity.cross(angular_velocity.cross(imu_offset_));

      accel += acc_offset; // 修正後的加速度

      aligned_msg.linear_acceleration.x = accel.x();
      aligned_msg.linear_acceleration.y = accel.y();
      aligned_msg.linear_acceleration.z = accel.z();

      pub_imu_->publish(aligned_msg);
  }
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  tf2::Matrix3x3 mat_;
  tf2::Vector3 imu_offset_; // IMU 偏移位置
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImuAligner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
