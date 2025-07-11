// // Nav2 (/cmd_vel)
// //     ↓
// // controller_manager + diff_drive_controller
// //     ↓
// // MotorSet::write()   → 發送 rpm 給馬達
// //     ↓                    ↑
// // 馬達運轉                MotorSet::read()  → 更新位置與速度
// //     ↓                    ↑
// // wheel_position_   ←   馬達編碼器數據
// // wheel_velocity_   ←   馬達速度 (RPM)
// //     ↓
// // controller_manager + diff_drive_controller (里程計計算)
// //     ↓
// // /odom → Nav2

// #include "motor_base/motor_set.h"
// #include "pluginlib/class_list_macros.hpp"
// // #include "sensor_msgs/msg/imu.hpp"
// #include "hardware_interface/types/hardware_interface_type_values.hpp"

// struct PID {
//   double kp;
//   double ki;
//   double kd;
//   double integral;
//   double prev_err;
// };

// // ← 左右各一個，全域靜態或加到 MotorSet 成員都行
// static PID pid_L {1.2, 0.00, 0.07, 0.0, 0.0};   // 左輪群：前左 + 後左
// static PID pid_R {1.2, 0.00, 0.07, 0.0, 0.0};   // 右輪群：前右 + 後右

// // 取樣週期（跟 ros2_control period 同步）；如你用 50 Hz 就是 0.02 s
// static const double CONTROL_DT = 0.02;      // 秒
// static const double GEAR_RATIO = 1.96;      // 你馬達減速比
// static const double MAX_RPM    = 1500.0;    // 馬達極速，用來限幅

// namespace motor_base
// {


// //回傳型態 類別名稱::函式名稱(參數)
// hardware_interface::CallbackReturn MotorSet::on_init(const hardware_interface::HardwareInfo &info)
// {
//     RCLCPP_INFO(rclcpp::get_logger("MotorSet"), "馬達 init() 啟動");

//     // 準備變數
//     wheel_radius_ = std::stod(info.hardware_parameters.at("wheel_radius"));
//     Lx_ = std::stod(info.hardware_parameters.at("Lx"));
//     Ly_ = std::stod(info.hardware_parameters.at("Ly"));

//     // 初始化 ZLAC 馬達
//     motorBC_ = std::make_unique<ZLAC>();
//     motorAD_ = std::make_unique<ZLAC>();

//     wheel_position_.resize(2, 0.0); //讀取的位置
//     wheel_velocity_.resize(2, 0.0); //讀取的速度
//     wheel_command_.resize(2, 0.0); //接收 /cmd_vel 的速度指令

//     encoder_A_prev_ = encoder_B_prev_ = encoder_C_prev_ = encoder_D_prev_ = 0;



//     return hardware_interface::CallbackReturn::SUCCESS;


// }

// // ros2_control 啟動時會執行的函式 開啟馬達
// hardware_interface::CallbackReturn MotorSet::on_configure(const rclcpp_lifecycle::State &)
// {
//     RCLCPP_INFO(rclcpp::get_logger("MotorSet"), "MotorSet on_configure() executed.");
//     // 初始化 ZLAC 馬達
//     motorBC_->init("/dev/motorttyUSB0", 115200, 0x01, false);
//     motorAD_->init("/dev/motorttyUSB1", 115200, 0x01, false);

//     const uint16_t ACC = 100;   // ms
//     const uint16_t DEC = 80;   // ms

//     // 前輪板
//     motorBC_->set_acc_time(ACC, "LEFT");
//     motorBC_->set_acc_time(ACC, "RIGHT");
//     motorBC_->set_decc_time(DEC, "LEFT");
//     motorBC_->set_decc_time(DEC, "RIGHT");

//     // 後輪板：先跟前輪同速，試車再視需要一起+10~20 ms
//     motorAD_->set_acc_time(ACC, "LEFT");
//     motorAD_->set_acc_time(ACC, "RIGHT");
//     motorAD_->set_decc_time(DEC, "LEFT");
//     motorAD_->set_decc_time(DEC, "RIGHT");

//     if(motorBC_->enable() != 0 || motorAD_->enable() != 0)
//     {
//         RCLCPP_ERROR(rclcpp::get_logger("MotorSet"), "馬達啟動失敗！");
//         return hardware_interface::CallbackReturn::ERROR;
//     }
//     return hardware_interface::CallbackReturn::SUCCESS;
// }

// //停止函式 關閉馬達
// hardware_interface::CallbackReturn MotorSet::on_deactivate(const rclcpp_lifecycle::State &)
// {
//     motorBC_->terminate();
//     motorAD_->terminate();
//     return hardware_interface::CallbackReturn::SUCCESS;
// }


// hardware_interface::CallbackReturn MotorSet::on_activate(const rclcpp_lifecycle::State &)
// {
//     RCLCPP_INFO(rclcpp::get_logger("MotorSet"), "MotorSet on_activate() executed.");

//     rclcpp::sleep_for(std::chrono::milliseconds(500));
   
//     return hardware_interface::CallbackReturn::SUCCESS;
// }


// std::vector<hardware_interface::StateInterface> MotorSet::export_state_interfaces()
// {
//     std::vector<hardware_interface::StateInterface> state_interfaces;

//     state_interfaces.emplace_back("left_wheel_joint", hardware_interface::HW_IF_POSITION, &wheel_position_[0]);
//     state_interfaces.emplace_back("left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &wheel_velocity_[0]);

//     state_interfaces.emplace_back("right_wheel_joint", hardware_interface::HW_IF_POSITION, &wheel_position_[1]);
//     state_interfaces.emplace_back("right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &wheel_velocity_[1]);

//     return state_interfaces;
// }

// std::vector<hardware_interface::CommandInterface> MotorSet::export_command_interfaces()
// {
//     std::vector<hardware_interface::CommandInterface> command_interfaces;

//     command_interfaces.emplace_back("left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &wheel_command_[0]);
//     command_interfaces.emplace_back("right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &wheel_command_[1]);

//     return command_interfaces;
// }
// // 讀取馬達數據函式
// hardware_interface::return_type MotorSet::read(const rclcpp::Time &time, const rclcpp::Duration &period)
// {
//     // 讀 RPM 來計算速度
//     MOT_DATA rpm_BC = motorBC_->get_rpm();
//     MOT_DATA rpm_AD = motorAD_->get_rpm();

//     double rpm_B = static_cast<double>(rpm_BC.rpm_R); //左輪
//     double rpm_C = static_cast<double>(rpm_BC.rpm_L); //右輪
//     double rpm_A = static_cast<double>(rpm_AD.rpm_R); //左輪
//     double rpm_D = static_cast<double>(rpm_AD.rpm_L); //右輪
//     double gear_ratio = 1.96;

//     double rpm_left  = 0.5 * (rpm_A + rpm_B);
//     double rpm_right = 0.5 * (rpm_D+ rpm_C);

//     wheel_velocity_[0] = (rpm_left / gear_ratio) * 2.0 * M_PI / 60.0;
//     wheel_velocity_[1] = (rpm_right / gear_ratio) * 2.0 * M_PI / 60.0;

//     double dt = period.seconds(); // 控制週期時間

//     wheel_position_[0] += wheel_velocity_[0] * dt; // 左輪
//     wheel_position_[1] += wheel_velocity_[1] * dt; // 右輪
    

//     RCLCPP_INFO(rclcpp::get_logger("MotorSet"),
//         "RPM B: %.2f, C: %.2f, Left Position: %.4f, Right Position: %.4f",
//         rpm_B, rpm_C, wheel_position_[0], wheel_position_[1]);



//     return hardware_interface::return_type::OK;

    
// }

// // 發送馬達速度函式，每個控制週期會呼叫，將速度指令轉成RPM發送給馬達
// hardware_interface::return_type MotorSet::write(const rclcpp::Time &, const rclcpp::Duration &period)
// {
//     // RCLCPP_INFO(rclcpp::get_logger("MotorSet"), "接收到速度指令：L %.3f, R %.3f", wheel_command_[0], wheel_command_[1]);
//     int16_t cmd_rpm_L = static_cast<int16_t>(wheel_command_[0] * 60.0 / (2 * M_PI));
//     int16_t cmd_rpm_R = static_cast<int16_t>(wheel_command_[1] * 60.0 / (2 * M_PI));

//     int16_t act_rpm_L = static_cast<int16_t>(wheel_velocity_[0] * 60.0 / (2 * M_PI));
//     int16_t act_rpm_R = static_cast<int16_t>(wheel_velocity_[1] * 60.0 / (2 * M_PI));

//     double dt = period.seconds();

//     auto pid_step = [dt](PID &c, double ref, double meas){
//         double err = ref - meas;
//         c.integral  += err * dt;
//         double deriv = (err - c.prev_err) / dt;
//         c.prev_err   = err;
//         return c.kp * err + c.ki * c.integral + c.kd * deriv;
//     };

//     double out_L = pid_step(pid_L, cmd_rpm_L, act_rpm_L);
//     double out_R = pid_step(pid_R, cmd_rpm_R, act_rpm_R);

//     auto clamp = [](double v){
//         if (v >  MAX_RPM) return  MAX_RPM;
//         if (v < -MAX_RPM) return -MAX_RPM;
//         return v;
//     };

//     int16_t rpm_L = static_cast<int16_t>(clamp(out_L));
//     int16_t rpm_R = static_cast<int16_t>(clamp(out_R));

//     motorBC_->set_double_rpm(rpm_R, rpm_L);
//     motorAD_->set_double_rpm(rpm_R, rpm_L);

//     // RCLCPP_DEBUG(rclcpp::get_logger("MotorSet"), "Sent RPM: A: %d, B: %d, C: %d, D: %d",
//     //     speed_A, speed_B, speed_C, speed_D);

//     return hardware_interface::return_type::OK;
// }

// }

// // 插件導出，告訴ros2_control這個類別是硬體接口插件
// PLUGINLIB_EXPORT_CLASS(motor_base::MotorSet, hardware_interface::SystemInterface);
