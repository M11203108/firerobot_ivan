#ifndef MOTOR_SET_H
#define MOTOR_SET_H

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include "zlac8015d.h" // 這是你的 ZLAC 驅動馬達的程式碼

namespace motor_base
{
class MotorSet : public hardware_interface::SystemInterface
{
public:
    // ros2_control 初始化時會呼叫，拿來設定參數用
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    // 啟動前配置硬體，像是開串口、準備好馬達
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    // 啟動馬達
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    // 關閉馬達
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    // 每個控制週期都會呼叫，讀取馬達的「編碼器位置 & 速度」
    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    // 每個控制週期都會呼叫，把目標速度發送給馬達
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;


private:
    std::shared_ptr<ZLAC> motorBC_;  // 控制 右前C 左前B
    std::shared_ptr<ZLAC> motorAD_;  // 控制 右後D 左後A

    // 輪子的位置、速度、速度命令
    std::vector<double> wheel_position_;
    std::vector<double> wheel_velocity_;
    std::vector<double> wheel_command_;

    // 上一次編碼器數據，計算增量用
    int32_t encoder_A_prev_;
    int32_t encoder_B_prev_;
    int32_t encoder_C_prev_;
    int32_t encoder_D_prev_;

    // 機器人底盤參數
    double wheel_radius_;
    double Lx_;
    double Ly_;
};
}  // namespace motor_base

#endif
