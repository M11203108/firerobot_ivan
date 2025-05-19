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
// #include "sensor_msgs/msg/imu.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace motor_base
{


//回傳型態 類別名稱::函式名稱(參數)
hardware_interface::CallbackReturn MotorSet::on_init(const hardware_interface::HardwareInfo &info)
{
    RCLCPP_INFO(rclcpp::get_logger("MotorSet"), "馬達 init() 啟動");

    // 準備變數
    wheel_radius_ = std::stod(info.hardware_parameters.at("wheel_radius"));
    Lx_ = std::stod(info.hardware_parameters.at("Lx"));
    Ly_ = std::stod(info.hardware_parameters.at("Ly"));

    // 初始化 ZLAC 馬達
    motorBC_ = std::make_unique<ZLAC>();
    motorAD_ = std::make_unique<ZLAC>();

    wheel_position_.resize(2, 0.0); //讀取的位置
    wheel_velocity_.resize(2, 0.0); //讀取的速度
    wheel_command_.resize(2, 0.0); //接收 /cmd_vel 的速度指令

    encoder_A_prev_ = encoder_B_prev_ = encoder_C_prev_ = encoder_D_prev_ = 0;



    return hardware_interface::CallbackReturn::SUCCESS;


}

// ros2_control 啟動時會執行的函式 開啟馬達
hardware_interface::CallbackReturn MotorSet::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("MotorSet"), "MotorSet on_configure() executed.");
    // 初始化 ZLAC 馬達
    motorBC_->init("/dev/motorttyUSB0", 115200, 0x01, false);
    motorAD_->init("/dev/motorttyUSB1", 115200, 0x01, false);

    if(motorBC_->enable() != 0 || motorAD_->enable() != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("MotorSet"), "馬達啟動失敗！");
        return hardware_interface::CallbackReturn::ERROR;
    }
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

    rclcpp::sleep_for(std::chrono::milliseconds(500));
   
    return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> MotorSet::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back("left_wheel_joint", hardware_interface::HW_IF_POSITION, &wheel_position_[0]);
    state_interfaces.emplace_back("left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &wheel_velocity_[0]);

    state_interfaces.emplace_back("right_wheel_joint", hardware_interface::HW_IF_POSITION, &wheel_position_[1]);
    state_interfaces.emplace_back("right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &wheel_velocity_[1]);

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MotorSet::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back("left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &wheel_command_[0]);
    command_interfaces.emplace_back("right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &wheel_command_[1]);

    return command_interfaces;
}
// 讀取馬達數據函式
hardware_interface::return_type MotorSet::read(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    // 讀 RPM 來計算速度
    MOT_DATA rpm_BC = motorBC_->get_rpm();

    double rpm_B = static_cast<double>(rpm_BC.rpm_R); //左輪
    double rpm_C = static_cast<double>(rpm_BC.rpm_L); //右輪

    wheel_velocity_[0] = rpm_B * 2.0 * M_PI / 60.0;
    wheel_velocity_[1] = rpm_C * 2.0 * M_PI / 60.0;

    double dt = period.seconds(); // 控制週期時間

    wheel_position_[0] += wheel_velocity_[0] * dt; // 左輪
    wheel_position_[1] += wheel_velocity_[1] * dt; // 右輪
    

    RCLCPP_INFO(rclcpp::get_logger("MotorSet"),
        "RPM B: %.2f, C: %.2f, Left Position: %.4f, Right Position: %.4f",
        rpm_B, rpm_C, wheel_position_[0], wheel_position_[1]);



    return hardware_interface::return_type::OK;

    
}

// 發送馬達速度函式，每個控制週期會呼叫，將速度指令轉成RPM發送給馬達
hardware_interface::return_type MotorSet::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    // RCLCPP_INFO(rclcpp::get_logger("MotorSet"), "接收到速度指令：L %.3f, R %.3f", wheel_command_[0], wheel_command_[1]);
    int16_t speed_B = static_cast<int16_t>(wheel_command_[0] * 60.0 / (2 * M_PI * wheel_radius_));
    int16_t speed_C = static_cast<int16_t>(wheel_command_[1] * 60.0 / (2 * M_PI * wheel_radius_));

    int16_t speed_A = speed_B;  
    int16_t speed_D = speed_C;

    motorBC_->set_double_rpm(speed_C, speed_B);
    motorAD_->set_double_rpm(speed_D, speed_A);

    // RCLCPP_DEBUG(rclcpp::get_logger("MotorSet"), "Sent RPM: A: %d, B: %d, C: %d, D: %d",
    //     speed_A, speed_B, speed_C, speed_D);

    return hardware_interface::return_type::OK;
}

}

// 插件導出，告訴ros2_control這個類別是硬體接口插件
PLUGINLIB_EXPORT_CLASS(motor_base::MotorSet, hardware_interface::SystemInterface);
