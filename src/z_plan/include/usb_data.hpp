#ifndef USB_DATA_HPP
#define USB_DATA_HPP

#include <vector>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <serial/serial.h>

#pragma pack(push, 1)
struct nuc_rx_t {
    uint8_t SOF;
    uint8_t control_mode;
    float joint_theta[6];
    uint16_t CheckSum;
};
#pragma pack(pop)

class SerialRxNode : public rclcpp::Node {
public:
    SerialRxNode();
    void send_joint_angles(const float joint_angles[6], uint8_t control_mode = 0);
    void send_joint_angles(const std::vector<float>& joint_angles, uint8_t control_mode = 0);
    std::vector<float> get_joint_angles() const;
    void receive_data();
    void print_joint_angles() const;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr get_publisher() const;

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    std::vector<uint8_t> buffer_;
    serial::Serial serial_;
    const uint8_t EXPECTED_SOF = 0xAA;
    std::vector<float> joint_angle_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    uint16_t calculateCRC16(const uint8_t* data, size_t length);
    void process_buffer();
    bool checkCRC16(const std::vector<uint8_t>& buffer);
    void parse_buffer(const std::vector<uint8_t>& packet);
    void publish_joint_states(const std::vector<uint8_t>& packet);
};

#endif