// Nav2 (/cmd_vel)
//     ↓
// controller_manager + diff_drive_controller
//     ↓
// MotorSet::write()   → 發送 rpm 給馬達
//     ↓                    ↑
// 馬達運轉                MotorSet::read()  → 更新位置與速度
//     ↓                    ↑
// wheel_position_   ←   馬達編碼器數據
// wheel_velocity_   ←   馬達速度 (RPM)
//     ↓
// controller_manager + diff_drive_controller (里程計計算)
//     ↓
// /odom → Nav2

#include "motor_base/motor_set.h"
#include "pluginlib/class_list_macros.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace motor_base
{


//回傳型態 類別名稱::函式名稱(參數)
hardware_interface::CallbackReturn MotorSet::on_init(const hardware_interface::HardwareInfo &info)
{
    RCLCPP_INFO(rclcpp::get_logger("MotorSet"), "MotorSet init() executed.");
    // 準備變數
    wheel_radius_ = std::stod(info.hardware_parameters.at("wheel_radius"));
    Lx_ = std::stod(info.hardware_parameters.at("Lx"));
    Ly_ = std::stod(info.hardware_parameters.at("Ly"));

    // 初始化 ZLAC 馬達
    motorBC_ = std::make_shared<ZLAC>();
    motorAD_ = std::make_shared<ZLAC>();

    wheel_position_.resize(4, 0.0); //讀取的位置
    wheel_velocity_.resize(4, 0.0); //讀取的速度
    wheel_command_.resize(4, 0.0); //接收 /cmd_vel 的速度指令

    encoder_A_prev_ = encoder_B_prev_ = encoder_C_prev_ = encoder_D_prev_ = 0;
    return hardware_interface::CallbackReturn::SUCCESS;


}

// ros2_control 啟動時會執行的函式 開啟馬達
hardware_interface::CallbackReturn MotorSet::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("MotorSet"), "MotorSet on_configure() executed.");
    // 初始化 ZLAC 馬達
    motorBC_->init("/dev/motorttyUSB0", 115200, 0x01, true);
    motorAD_->init("/dev/motorttyUSB1", 115200, 0x01, true);
    return hardware_interface::CallbackReturn::SUCCESS;
}

//停止函式 關閉馬達
hardware_interface::CallbackReturn MotorSet::on_deactivate(const rclcpp_lifecycle::State &)
{
    motorBC_->terminate();
    motorAD_->terminate();
    return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn MotorSet::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("MotorSet"), "MotorSet on_activate() executed.");
    return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> MotorSet::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back("left_front_joint", hardware_interface::HW_IF_POSITION, &wheel_position_[0]);
    state_interfaces.emplace_back("left_front_joint", hardware_interface::HW_IF_VELOCITY, &wheel_velocity_[0]);

    state_interfaces.emplace_back("right_front_joint", hardware_interface::HW_IF_POSITION, &wheel_position_[1]);
    state_interfaces.emplace_back("right_front_joint", hardware_interface::HW_IF_VELOCITY, &wheel_velocity_[1]);

    state_interfaces.emplace_back("left_wheel_joint", hardware_interface::HW_IF_POSITION, &wheel_position_[2]);
    state_interfaces.emplace_back("left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &wheel_velocity_[2]);

    state_interfaces.emplace_back("right_wheel_joint", hardware_interface::HW_IF_POSITION, &wheel_position_[3]);
    state_interfaces.emplace_back("right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &wheel_velocity_[3]);

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MotorSet::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back("left_front_joint", hardware_interface::HW_IF_VELOCITY, &wheel_command_[0]);
    command_interfaces.emplace_back("right_front_joint", hardware_interface::HW_IF_VELOCITY, &wheel_command_[1]);
    command_interfaces.emplace_back("left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &wheel_command_[2]);
    command_interfaces.emplace_back("right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &wheel_command_[3]);

    return command_interfaces;
}
// 讀取馬達數據函式
hardware_interface::return_type MotorSet::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    // 讀取編碼器數據
    MOT_DATA data_BC = motorBC_->get_position();
    MOT_DATA data_AD = motorAD_->get_position();
    RCLCPP_INFO(rclcpp::get_logger("MotorSet"), "Encoder A: %d, B: %d, C: %d, D: %d",
            data_AD.encoder_R, data_BC.encoder_R, data_BC.encoder_L, data_AD.encoder_L);

    int32_t delta_A = data_AD.encoder_R - encoder_A_prev_;
    int32_t delta_B = data_BC.encoder_R - encoder_B_prev_;
    int32_t delta_C = data_BC.encoder_L - encoder_C_prev_;
    int32_t delta_D = data_AD.encoder_L - encoder_D_prev_;

    encoder_A_prev_ = data_AD.encoder_R;
    encoder_B_prev_ = data_BC.encoder_R;
    encoder_C_prev_ = data_BC.encoder_L;
    encoder_D_prev_ = data_AD.encoder_L;

    double distance_per_pulse = (2 * M_PI * wheel_radius_) / 3600.0; // 3600 為編碼器分辨率

    // 根據編碼器變化量計算每個輪子的位移 (位置)，用於里程計
    wheel_position_[0] += delta_B * distance_per_pulse; // 左前B
    wheel_position_[1] += delta_C * distance_per_pulse; // 右前C
    wheel_position_[2] += delta_A * distance_per_pulse; // 左後A
    wheel_position_[3] += delta_D * distance_per_pulse; // 右後D

    // 讀 RPM 來計算速度
    MOT_DATA rpm_BC = motorBC_->get_rpm();
    MOT_DATA rpm_AD = motorAD_->get_rpm();
    RCLCPP_INFO(rclcpp::get_logger("MotorSet"), "RPM A: %d, B: %d, C: %d, D: %d",
            rpm_AD.rpm_R, rpm_BC.rpm_R, rpm_BC.rpm_L, rpm_AD.rpm_L);

    double rpm_A = static_cast<double>(rpm_AD.rpm_R);
    double rpm_B = static_cast<double>(rpm_BC.rpm_R);
    double rpm_C = static_cast<double>(rpm_BC.rpm_L);
    double rpm_D = static_cast<double>(rpm_AD.rpm_L);

    wheel_velocity_[0] = rpm_B * 2 * M_PI * wheel_radius_ / 60.0;
    wheel_velocity_[1] = rpm_C * 2 * M_PI * wheel_radius_ / 60.0;
    wheel_velocity_[2] = rpm_A * 2 * M_PI * wheel_radius_ / 60.0;
    wheel_velocity_[3] = rpm_D * 2 * M_PI * wheel_radius_ / 60.0;

    return hardware_interface::return_type::OK;
}

// 發送馬達速度函式，每個控制週期會呼叫，將速度指令轉成RPM發送給馬達
hardware_interface::return_type MotorSet::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    int16_t speed_B = int16_t(wheel_command_[0] * (30 / M_PI / wheel_radius_));
    int16_t speed_C = int16_t(wheel_command_[1] * (30 / M_PI / wheel_radius_));
    int16_t speed_A = int16_t(wheel_command_[2] * (30 / M_PI / wheel_radius_));
    int16_t speed_D = int16_t(wheel_command_[3] * (30 / M_PI / wheel_radius_));

    motorBC_->set_double_rpm(speed_C, speed_B);
    motorAD_->set_double_rpm(speed_D, speed_A);

    return hardware_interface::return_type::OK;
}

}

// 插件導出，告訴ros2_control這個類別是硬體接口插件
PLUGINLIB_EXPORT_CLASS(motor_base::MotorSet, hardware_interface::SystemInterface);
