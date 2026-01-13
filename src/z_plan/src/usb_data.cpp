#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include "usb_data.hpp"

using namespace std::chrono_literals;

SerialRxNode::SerialRxNode() : Node("simple_robot_receiver"){
    try {
        serial_.setPort("/dev/ttyUSB0");  // 串口号
        serial_.setBaudrate(115200);
        serial_.open();
        RCLCPP_INFO(this->get_logger(), "serial open success");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "serial open failed: %s", e.what());
    }
    
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/usb_joint_states", 10);
}

void SerialRxNode::send_joint_angles(const float joint_angles[6], uint8_t control_mode){
        nuc_rx_t tx_data;
        tx_data.SOF = EXPECTED_SOF;
        tx_data.control_mode = control_mode;
        for(int i = 0; i < 6; ++i){
            tx_data.joint_theta[i] = joint_angles[i];
        }
        
        // 计算CRC16校验：对所有字节（除了CheckSum）计算CRC16
        uint8_t* data_ptr = reinterpret_cast<uint8_t*>(&tx_data);
        size_t data_length = sizeof(nuc_rx_t) - sizeof(uint16_t);  // 排除CheckSum字段
        tx_data.CheckSum = calculateCRC16(data_ptr, data_length);
        
        try {
            serial_.write(data_ptr, sizeof(nuc_rx_t));
            RCLCPP_DEBUG(this->get_logger(), "Sent joint angles to serial port");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send joint angles: %s", e.what());
        }
}

void SerialRxNode::send_joint_angles(const std::vector<float>& joint_angles, uint8_t control_mode){
        if(joint_angles.size() != 6){
            RCLCPP_ERROR(this->get_logger(), "Joint angles vector must have 6 elements");
            return;
        }
        send_joint_angles(joint_angles.data(), control_mode);
}

std::vector<float> SerialRxNode::get_joint_angles() const {
        return joint_angle_;
}

// 串口接收数据（需要在外部线程中调用）
void SerialRxNode::receive_data(){
        while(rclcpp::ok()){
            if(serial_.available()){
                std::string raw_data = serial_.read(serial_.available());
                std::vector<uint8_t> new_data(raw_data.begin(), raw_data.end());
                buffer_.insert(buffer_.end(), new_data.begin(), new_data.end());
                process_buffer();
            }
        }
}

void SerialRxNode::print_joint_angles() const {
    RCLCPP_INFO(this->get_logger(), "Joint Angles:");
    for (size_t i = 0; i < joint_angle_.size(); ++i) {
        RCLCPP_INFO(this->get_logger(), "joint[%zu]: %f", i + 1, joint_angle_[i]);
    }
}

rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr SerialRxNode::get_publisher() const {
    return joint_pub_;
}

void SerialRxNode::process_buffer(){
        RCLCPP_INFO(this->get_logger(), "Processing serial buffer");
        const size_t packet_size = sizeof(nuc_rx_t);
        while(buffer_.size() >= packet_size){
            size_t sof_index = 0;
            for (; sof_index < buffer_.size(); ++sof_index){
                if(buffer_[sof_index] == EXPECTED_SOF){
                    break;
                }
            }
            if (sof_index >= buffer_.size()){
                buffer_.clear();
                return;
            }
            if (sof_index > 0){
                buffer_.erase(buffer_.begin(), buffer_.begin() + sof_index);
            }
            if(buffer_.size() < packet_size){
                return;
            }
            std::vector<uint8_t> packet(buffer_.begin(), buffer_.begin() + packet_size);
            if(checkCRC16(packet)){
                RCLCPP_INFO(this->get_logger(), "parse serial data");
                parse_buffer(packet);
                publish_joint_states(packet); // Pass the packet as argument
            }
            buffer_.erase(buffer_.begin(), buffer_.begin() + packet_size);
        }        
}   

bool SerialRxNode::checkCRC16(const std::vector<uint8_t>& buffer){
        size_t data_length = sizeof(nuc_rx_t) - sizeof(uint16_t);
        uint16_t calculated_crc = calculateCRC16(buffer.data(), data_length);
        const nuc_rx_t* rx_data = reinterpret_cast<const nuc_rx_t*>(buffer.data());
        uint16_t received_crc = rx_data->CheckSum;
        return calculated_crc == received_crc;
}

void SerialRxNode::parse_buffer(const std::vector<uint8_t>& packet){
        const nuc_rx_t* rx_data = reinterpret_cast<const nuc_rx_t*>(packet.data());
        for(int i = 0; i < 6; ++i){
            joint_angle_[i] = rx_data->joint_theta[i];
        }
}

void SerialRxNode::publish_joint_states(const std::vector<uint8_t>& /*packet*/){
    auto message = sensor_msgs::msg::JointState();
    message.header.stamp = this->now();
    message.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

    // Convert joint_angle_ to std::vector<double>
    std::vector<double> joint_angle_double(joint_angle_.begin(), joint_angle_.end());
    message.position = joint_angle_double;

    joint_pub_->publish(message);
}

// CRC16-CCITT 计算函数（多项式: 0x1021, 初始值: 0xFFFF）
uint16_t SerialRxNode::calculateCRC16(const uint8_t* data, size_t length){
        uint16_t crc = 0xFFFF;  // 初始值
        const uint16_t polynomial = 0x1021;  // CRC-16-CCITT多项式
        
        for(size_t i = 0; i < length; ++i){
            crc ^= (static_cast<uint16_t>(data[i]) << 8);
            for(int j = 0; j < 8; ++j){
                if(crc & 0x8000){
                    crc = (crc << 1) ^ polynomial;
                } else {
                    crc <<= 1;
                }
            }
        }
        return crc;
}

