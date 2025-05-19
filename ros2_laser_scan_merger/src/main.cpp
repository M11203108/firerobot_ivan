// scan_merger_node.cpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>

#include <string>
#include <vector>
#include <cmath>
#include <limits>

class scanMerger : public rclcpp::Node
{
public:
    scanMerger() 
      : Node("ros2_laser_scan_merger"),
        tf_buffer_(this->get_clock()),  // 初始化 tf_buffer_
        tf_listener_(tf_buffer_)
    {
        initialize_params();
        refresh_params();

        laser1_ = nullptr;
        laser2_ = nullptr;

        auto default_qos = rclcpp::QoS(rclcpp::SensorDataQoS());

        // 创建订阅者
        sub1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            topic1_, default_qos, std::bind(&scanMerger::scan_callback1, this, std::placeholders::_1));
        sub2_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            topic2_, default_qos, std::bind(&scanMerger::scan_callback2, this, std::placeholders::_1));

        // 创建发布者
        merged_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(outputTopic_, rclcpp::SystemDefaultsQoS());

        RCLCPP_INFO(this->get_logger(), "LaserScan Merger Node Initialized");
    }

private:
    void scan_callback1(const sensor_msgs::msg::LaserScan::SharedPtr _msg)
    {
        laser1_ = _msg;
        update_merged_laserscan();
    }

    void scan_callback2(const sensor_msgs::msg::LaserScan::SharedPtr _msg)
    {
        laser2_ = _msg;
        update_merged_laserscan();
    }

    void update_merged_laserscan()
    {
        // 检查是否有数据
        if (!laser1_ || !laser2_)
        {
            return;
        }

        auto merged_scan = std::make_shared<sensor_msgs::msg::LaserScan>();

        // 设置 merged_scan 的元数据信息
        merged_scan->header.stamp = this->now();
        merged_scan->header.frame_id = target_frame_;
        merged_scan->angle_min = -M_PI;
        merged_scan->angle_max = M_PI;
        merged_scan->angle_increment = 0.0058; // 根据需要调整
        merged_scan->time_increment = 0.0;
        merged_scan->scan_time = 0.1;
        merged_scan->range_min = std::min(laser1_->range_min, laser2_->range_min);
        merged_scan->range_max = std::max(laser1_->range_max, laser2_->range_max);

        int total_points = static_cast<int>((merged_scan->angle_max - merged_scan->angle_min) / merged_scan->angle_increment);
        merged_scan->ranges.resize(total_points, std::numeric_limits<float>::infinity());
        merged_scan->intensities.resize(total_points, 0.0);

        // 合并激光雷达数据
        merge_single_scan(laser1_, merged_scan);
        merge_single_scan(laser2_, merged_scan);

        // 发布合并后的 LaserScan
        merged_scan_pub_->publish(*merged_scan);
    }

    void merge_single_scan(const sensor_msgs::msg::LaserScan::SharedPtr& scan,
                           sensor_msgs::msg::LaserScan::SharedPtr& merged_scan)
    {
        // 获取变换
        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            // 获取从 scan->header.frame_id 到 target_frame_ 的变换
            transform_stamped = tf_buffer_.lookupTransform(
                target_frame_, scan->header.frame_id, tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform from %s to %s: %s",
                        scan->header.frame_id.c_str(), target_frame_.c_str(), ex.what());
            return;
        }

        int num_ranges = scan->ranges.size();
        float angle = scan->angle_min;

        for (int i = 0; i < num_ranges; ++i)
        {
            float range = scan->ranges[i];

            
            if (std::isfinite(range) && range >= 0.1)
            {
                // 将极坐标转换为笛卡尔坐标
                float x = range * std::cos(angle);
                float y = range * std::sin(angle);

                // 创建点
                geometry_msgs::msg::PointStamped point_in, point_out;
                point_in.header.frame_id = scan->header.frame_id;
                point_in.point.x = x;
                point_in.point.y = y;
                point_in.point.z = 0.0;

                // 应用变换
                try {
                    tf2::doTransform(point_in, point_out, transform_stamped);
                } catch (tf2::TransformException &ex) {
                    RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
                    return;
                }

                // 转换回极坐标
                float merged_range = std::hypot(point_out.point.x, point_out.point.y);
                float merged_angle = std::atan2(point_out.point.y, point_out.point.x);

                // 将角度归一化到 [-PI, PI]
                if (merged_angle < -M_PI)
                    merged_angle += 2 * M_PI;
                if (merged_angle > M_PI)
                    merged_angle -= 2 * M_PI;

                if (merged_angle >= merged_scan->angle_min && merged_angle <= merged_scan->angle_max)
                {
                    // 计算索引
                    int merged_index = static_cast<int>((merged_angle - merged_scan->angle_min) / merged_scan->angle_increment);
                    if (merged_index >= 0 && merged_index < static_cast<int>(merged_scan->ranges.size()))
                    {
                        // 如果当前范围更小，则更新
                        if (merged_range < merged_scan->ranges[merged_index])
                        {
                            merged_scan->ranges[merged_index] = merged_range;
                            merged_scan->intensities[merged_index] = scan->intensities[i];
                        }
                    }
                }
            }

            angle += scan->angle_increment;
        }
    }

    void initialize_params()
    {
        // 声明参数
        this->declare_parameter<std::string>("scanTopic1", "/lidar_1/scan");
        this->declare_parameter<std::string>("scanTopic2", "/lidar_2/scan");
        this->declare_parameter<std::string>("outputTopic", "/lidar_merge_scan");
        this->declare_parameter<std::string>("target_frame", "base_footprint");
    }

    void refresh_params()
    {
        // 获取参数
        this->get_parameter("scanTopic1", topic1_);
        this->get_parameter("scanTopic2", topic2_);
        this->get_parameter("outputTopic", outputTopic_);
        this->get_parameter("target_frame", target_frame_);
    }

    // 成员变量
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub1_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub2_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr merged_scan_pub_;

    sensor_msgs::msg::LaserScan::SharedPtr laser1_;
    sensor_msgs::msg::LaserScan::SharedPtr laser2_;

    std::string topic1_, topic2_, outputTopic_;

    // 添加 TF 相关的成员变量
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::string target_frame_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<scanMerger>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
