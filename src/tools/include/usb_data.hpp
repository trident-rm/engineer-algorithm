#ifndef USB_DATA_HPP
#define USB_DATA_HPP
#include <vector>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <serial/serial.h>

/**
 * @brief 角度低通滤波器
 * 用于平滑机械臂关节角度的变化
 */
class AngleFilter {
public:
    AngleFilter(size_t size, double alpha) 
        : filtered_angles_(size, 0.0), alpha_(alpha), initialized_(false) {}
    
    std::vector<double> filter(const std::vector<float>& raw_angles) {
        std::vector<double> angles(raw_angles.begin(), raw_angles.end());
        
        if (!initialized_) {
            filtered_angles_ = angles;
            initialized_ = true;
        } else {
            for (size_t i = 0; i < angles.size() && i < filtered_angles_.size(); ++i) {
                filtered_angles_[i] = alpha_ * angles[i] + (1.0 - alpha_) * filtered_angles_[i];
            }
        }
        return filtered_angles_;
    }
    
private:
    std::vector<double> filtered_angles_;
    double alpha_;
    bool initialized_;
};

#pragma pack(push, 1)
struct roboarm_with_controller_rx_t {
    uint8_t SOF;
    uint8_t control_mode;
    int16_t robo_arm_joint_theta[6];
    int16_t controller_joint_theta[6];
    uint16_t CheckSum;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct roboarm_with_controller_tx_t {
    uint8_t SOF;
    int16_t robo_arm_joint_theta[6];
    float current[6];
    uint16_t CheckSum;
};
#pragma pack(pop)

class SerialRxNode : public rclcpp::Node {
public:
    SerialRxNode();
    void send_roboarm_with_controller(const std::vector<float>& currents, const std::vector<float>& robo_arm_angles);
    std::vector<float> get_joint_angles() const;
    std::vector<float> get_robo_arm_angles() const;
    std::vector<float> get_controller_angles() const;
    std::vector<float> get_joint_currents() const;
    uint8_t get_control_mode() const;
    
    void print_joint_angles() const;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr get_publisher() const;

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr robo_arm_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr controller_pub_;
    rclcpp::TimerBase::SharedPtr read_timer_; 
    void read_serial_callback();
    std::vector<uint8_t> buffer_;
    serial::Serial serial_;
    const uint8_t EXPECTED_SOF = 0x91;
    std::vector<float> joint_angle_{-0.0116f, -0.2577f, 1.00f, -3.1592f, 6.4611f, -1.5546f};
    std::vector<float> joint_current_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    std::vector<float> robo_arm_angle_{1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    std::vector<float> controller_angle_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t control_mode_{0};
    
    // 角度滤波器
    AngleFilter robo_filter_{6, 0.3};
    
    void process_buffer();
    uint32_t verify_CRC16_check_sum(uint8_t *pchMessage, uint32_t dwLength);
    uint16_t get_CRC16_check_sum(uint8_t *pch_message,uint32_t dw_length,uint16_t wCRC);
    void parse_buffer(const std::vector<uint8_t>& packet);
    void publish_joint_states(const std::vector<uint8_t>& packet);
};

#endif
