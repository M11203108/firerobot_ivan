// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/imu.hpp>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Matrix3x3.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// class ImuAligner : public rclcpp::Node
// {
// public:
//   ImuAligner() : Node("imu_aligner")
//   {
//     sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
//       "/imu/data", 10,
//       std::bind(&ImuAligner::imuCallback, this, std::placeholders::_1));

//     pub_imu_ = create_publisher<sensor_msgs::msg::Imu>(
//       "/imu/data_aligned", 10);

//     // 將IMU原始座標系轉換成ROS標準座標系的旋轉矩陣
//     mat_.setValue(
//       0, 0, 1,   // X_new = Z_old (原本的前方)
//      -1, 0, 0,   // Y_new = -X_old (原本的右邊)
//       0, 1, 0    // Z_new = -Y_old (原本的下方)
//     );

//     imu_offset_.setValue(
//       0.4754, 0.13298, 0.9801
//     );
//   }

// private:
//   void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
//   {
//     sensor_msgs::msg::Imu aligned_msg = *msg;

//     // 轉換 Orientation
//     tf2::Quaternion q_orig, q_aligned;
//     tf2::fromMsg(msg->orientation, q_orig);

//     tf2::Vector3 x_axis(1, 0, 0), y_axis(0, 1, 0), z_axis(0, 0, 1);
//     x_axis = mat_ * x_axis;
//     y_axis = mat_ * y_axis;
//     z_axis = mat_ * z_axis;

//     tf2::Matrix3x3 new_mat(x_axis.x(), y_axis.x(), z_axis.x(),
//                            x_axis.y(), y_axis.y(), z_axis.y(),
//                            x_axis.z(), y_axis.z(), z_axis.z());

//     // new_mat.getRotation(q_aligned);
//     // aligned_msg.orientation = tf2::toMsg(q_aligned);

//     tf2::Quaternion q_orig, q_rotated;
//     tf2::fromMsg(msg->orientation, q_orig);

//     tf2::Quaternion q_mat;
//     mat_.getRotation(q_mat);

//     q_rotated = q_mat * q_orig;
//     q_rotated.normalize();

//     aligned_msg.orientation = tf2::toMsg(q_rotated);


//     // 轉換 Angular Velocity
//     tf2::Vector3 angular(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
//     angular = mat_ * angular;
//     aligned_msg.angular_velocity.x = angular.x();
//     aligned_msg.angular_velocity.y = angular.y();
//     aligned_msg.angular_velocity.z = angular.z();

//     // 轉換 Linear Acceleration
//     tf2::Vector3 accel(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
//     accel = mat_ * accel;
//     // 補償 IMU 偏移量對加速度的影響 (考慮角速度影響)
//     tf2::Vector3 angular_velocity(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
//     tf2::Vector3 acc_offset = angular_velocity.cross(angular_velocity.cross(imu_offset_));

//     accel += acc_offset; // 修正後的加速度
//     aligned_msg.linear_acceleration.x = accel.x();
//     aligned_msg.linear_acceleration.y = accel.y();
//     aligned_msg.linear_acceleration.z = accel.z();

//     pub_imu_->publish(aligned_msg);
//   }

//   rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
//   rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
//   tf2::Matrix3x3 mat_;
//   tf2::Vector3 imu_offset_;
// };

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<ImuAligner>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }
