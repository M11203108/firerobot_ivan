#include <rclcpp/rclcpp.hpp>
#include "zlac8015d.h"

class MotorTestNode : public rclcpp::Node {
public:
    MotorTestNode() : Node("motor_test_node") {
        RCLCPP_INFO(this->get_logger(), "初始化 ZLAC8015D 測試節點...");

        motorBC.init("/dev/motorttyUSB0", 115200, 0x01, true);
        motorAD.init("/dev/motorttyUSB1", 115200, 0x01, true);

        int bc_status = motorBC.enable();
        int ad_status = motorAD.enable();

        RCLCPP_INFO(this->get_logger(), "馬達 BC 啟動狀態: %d", bc_status);
        RCLCPP_INFO(this->get_logger(), "馬達 AD 啟動狀態: %d", ad_status);

        if (bc_status != 0 || ad_status != 0) {
            RCLCPP_ERROR(this->get_logger(), "馬達啟動失敗！");
        } else {
            test_motor();
        }
    }

private:
    ZLAC motorBC, motorAD;

    void test_motor() {
        RCLCPP_INFO(this->get_logger(), "發送測試速度指令: BC(100 RPM), AD(100 RPM)");
        motorBC.set_double_rpm(100, 100);
        motorAD.set_double_rpm(100, 100);

        // 等待馬達運行
        rclcpp::sleep_for(std::chrono::seconds(2));

        // 讀取馬達轉速
        MOT_DATA bc_data = motorBC.get_rpm();
        MOT_DATA ad_data = motorAD.get_rpm();

        RCLCPP_INFO(this->get_logger(), "讀取馬達轉速: BC (L: %.2f, R: %.2f), AD (L: %.2f, R: %.2f)",
                    bc_data.rpm_L, bc_data.rpm_R, ad_data.rpm_L, ad_data.rpm_R);

        // 測試停止馬達
        RCLCPP_INFO(this->get_logger(), "停止馬達...");
        motorBC.set_double_rpm(0, 0);
        motorAD.set_double_rpm(0, 0);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorTestNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
