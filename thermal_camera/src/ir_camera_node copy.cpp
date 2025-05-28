#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

extern "C" {
    #include <guideusb2livestream.h>
    #include <stdio.h>
    #include <sys/types.h>
    #include <unistd.h>
    #include <malloc.h>
    #include <string.h>
    #include "sys/time.h"
    #include "time.h"
    #include <libmemcached/memcached.h>
}

// **全局變數**
class IRCameraNode;
IRCameraNode *g_node_instance = nullptr;

// **靜態回調函數**
int frameCallbackStatic(guide_usb_frame_data_t *pVideoData);

class IRCameraNode : public rclcpp::Node
{
public:
    IRCameraNode() : Node("ir_camera")
    {
        g_node_instance = this; // 設定全局變數指向當前實例

        // **創建 ROS2 發佈者**
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/ir_camera/image", 10);
        max_temp_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/thermal_max_temp", 10);

        guide_usb_setloglevel(LOG_INFO);

        // **初始化 USB 設備**
        int ret = guide_usb_initial();
        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(), "❌ Initialization failed: %d. Try: sudo chmod -R 777 /dev/bus/usb", ret);
            rclcpp::shutdown();
            return;
        }

        // **開啟指令控制模式**
        ret = guide_usb_opencommandcontrol(serialCallback);
        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to open command control: %d", ret);
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "✅ guide_usb_getserialdata return: %d", ret);

        // **載入測量曲線**
        guide_measure_loadcurve();

        // **初始化 mDebugParam**
        mDebugParam = (guide_measure_debugparam_t *)malloc(sizeof(guide_measure_debugparam_t));
        if (!mDebugParam) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to allocate memory for mDebugParam.");
            rclcpp::shutdown();
            return;
        }

        // **設置裝置資訊**
        guide_usb_device_info_t deviceInfo;
        deviceInfo.width = 256;
        deviceInfo.height = 192;
        deviceInfo.video_mode = Y16_PARAM_YUV;

        // **開啟影像串流**
        ret = guide_usb_openstream(&deviceInfo, frameCallbackStatic, connectStatusCallback);
        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to open stream: %d", ret);
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "📸 熱像串流已啟動");

        // **確保節點持續運行**
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&IRCameraNode::keepAlive, this));
    }

    ~IRCameraNode()
    {
        guide_usb_closestream();
        guide_measure_deloadcurve();
        guide_usb_closecommandcontrol();
        guide_usb_exit();

        // **釋放 mDebugParam 記憶體**
        if (mDebugParam) {
            free(mDebugParam);
            mDebugParam = nullptr;
        }
    }

    // **影像與溫度處理**
    int frameCallback(guide_usb_frame_data_t *pVideoData)
    {
        if (!pVideoData || !pVideoData->frame_src_data || !mDebugParam)
            return 0;

        int width = pVideoData->frame_width;
        int height = pVideoData->frame_height;

        // 設定測量參數
        mDebugParam->exkf = 100;
        mDebugParam->exb = 0;
        mDebugParam->emiss = 98;
        mDebugParam->transs = 0;
        mDebugParam->reflectTemp = 23.0f;
        mDebugParam->distance = 30.0f;
        mDebugParam->fEnvironmentIncrement = 2500;

        // **影像轉換**
        cv::Mat img16(height, width, CV_16UC1, pVideoData->frame_src_data);
        cv::Mat img8;
        img16.convertTo(img8, CV_8UC1, 0.055); // 調整係數確保清晰度
        cv::Mat color_img;
        cv::applyColorMap(img8, color_img, cv::COLORMAP_JET); // **使用更清晰的顏色映射**
        // cv::normalize(img16, img16, 0, 65535, cv::NORM_MINMAX);

        // **發佈影像**
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", color_img).toImageMsg();
        msg->header.stamp = this->get_clock()->now();
        image_pub_->publish(*msg);

        // **尋找最高溫度**
        int max_index = 0;
        uint16_t max_gray_value = 0;

        for (int i = 0; i < width * height; i++) {
            if (pVideoData->frame_src_data[i] > max_gray_value) {
                max_gray_value = pVideoData->frame_src_data[i];
                max_index = i;
            }
        }

        int max_x = max_index % width;
        int max_y = max_index / width;

        // **單點溫度計算**
        float max_temp = guide_measure_convertsinglegray2temper(
            max_gray_value, pVideoData->paramLine, mDebugParam, 1);

        // **發佈最高溫度**
        std_msgs::msg::Float32MultiArray temp_msg;
        temp_msg.data = {static_cast<float>(max_x), static_cast<float>(max_y), max_temp};
        max_temp_pub_->publish(temp_msg);

        RCLCPP_INFO(this->get_logger(), "📡 發佈最高溫度: %.2f°C 位置: (%d, %d)", max_temp, max_x, max_y);

        return 1;
    }

    static int serialCallback(guide_usb_serial_data_t *pSerialData)
    {
        RCLCPP_INFO(rclcpp::get_logger("ir_camera"), "📡 收到序列數據，長度: %d", pSerialData->serial_recv_data_length);
        return 1;
    }

    static int connectStatusCallback(guide_usb_device_status_e deviceStatus)
    {
        if (deviceStatus == DEVICE_CONNECT_OK) {
            RCLCPP_INFO(rclcpp::get_logger("ir_camera"), "✅ 設備已連接！");
        } else {
            RCLCPP_WARN(rclcpp::get_logger("ir_camera"), "❌ 設備已斷開！");
        }
        return 1;
    }

    void keepAlive() {}

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr max_temp_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    guide_measure_debugparam_t *mDebugParam;  // **確保變數已宣告**
};

// **靜態回調函數**
int frameCallbackStatic(guide_usb_frame_data_t *pVideoData)
{
    return g_node_instance ? g_node_instance->frameCallback(pVideoData) : 0;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IRCameraNode>());
    rclcpp::shutdown();
    return 0;
}
