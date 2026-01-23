// Implementation of SerialRxNode methods. Class/struct definitions are in the header.
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include "usb_data.hpp"
#include <sstream>
#include <iomanip>
#include <cstring>

using namespace std::chrono_literals;

uint16_t CRC16_INIT = 0xffff;
const uint16_t wCRC_table[256] =
{
0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};


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
    uint32_t data_length = static_cast<uint32_t>(sizeof(nuc_rx_t) - sizeof(uint16_t));  // 排除CheckSum字段
    tx_data.CheckSum = this->get_CRC16_check_sum(data_ptr, data_length, CRC16_INIT);

    try {
        serial_.write(data_ptr, sizeof(nuc_rx_t));
        RCLCPP_INFO(this->get_logger(), "Sent joint angles to serial port");
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

#include <chrono>

void SerialRxNode::receive_data(){
    RCLCPP_INFO(this->get_logger(), "receive_data thread started");
    
    // 统计信息
    size_t total_bytes_received = 0;
    auto last_stat_time = std::chrono::steady_clock::now();
    size_t idle_counter = 0;
    
    while(rclcpp::ok()){
        try {
            // 检查串口是否打开
            if (!serial_.isOpen()) {
                RCLCPP_WARN(get_logger(), "Serial port not open, sleeping...");
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }
            
            // 检查是否有数据
            if(serial_.available()){
                RCLCPP_DEBUG(this->get_logger(), "Serial data available");
                size_t n = serial_.available();
                if (n > 0) {
                    std::string raw_data = serial_.read(n);
                    std::vector<uint8_t> new_data(raw_data.begin(), raw_data.end());
                    buffer_.insert(buffer_.end(), new_data.begin(), new_data.end());
                    
                    total_bytes_received += n;
                    
                    // 限制日志频率：每500ms最多一次
                    auto now = std::chrono::steady_clock::now();
                    static auto last_log_time = now;
                    if (now - last_log_time > std::chrono::milliseconds(500)) {
                        RCLCPP_DEBUG(this->get_logger(), "Received %zu bytes, total: %zu, buffer: %zu", 
                                    n, total_bytes_received, buffer_.size());
                        last_log_time = now;
                    }
                    
                    process_buffer();
                    
                    // 重置空闲计数器
                    idle_counter = 0;
                }
            } else {
                // 动态调整睡眠时间：空闲越久，睡眠越长
                size_t sleep_ms = 10;
                if (idle_counter > 1000) sleep_ms = 100;      // 空闲10秒后
                else if (idle_counter > 100) sleep_ms = 50;   // 空闲1秒后
                
                std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
                idle_counter++;
                
                // 每10秒打印一次空闲状态
                if (idle_counter % (10000 / sleep_ms) == 0) {  // 约10秒
                    RCLCPP_DEBUG(this->get_logger(), 
                               "Idle for %.1f seconds, buffer size: %zu, total received: %zu bytes",
                               idle_counter * sleep_ms / 1000.0, 
                               buffer_.size(),
                               total_bytes_received);
                }
            }
            
            // 定期打印统计信息（每30秒）
            auto now = std::chrono::steady_clock::now();
            if (now - last_stat_time > std::chrono::seconds(30)) {
                RCLCPP_DEBUG(get_logger(), 
                           "Serial statistics - Total bytes: %zu, Buffer size: %zu, Idle counter: %zu",
                           total_bytes_received, buffer_.size(), idle_counter);
                last_stat_time = now;
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Exception in receive_data: %s", e.what());
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "receive_data thread stopped");
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
    RCLCPP_DEBUG(this->get_logger(), "Processing serial buffer (size=%zu)", buffer_.size());
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

    if(this->verify_CRC16_check_sum(const_cast<uint8_t*>(packet.data()), static_cast<uint32_t>(packet.size()))){
            RCLCPP_DEBUG(this->get_logger(), "parse serial data");
            parse_buffer(packet);
            publish_joint_states(packet);
        } else {
            // 输出更多信息以便调试：包的十六进制和按两种字节序解析的 float
            std::ostringstream oss;
            oss << std::hex << std::setfill('0');
            for (size_t i = 0; i < packet.size(); ++i) {
                oss << std::setw(2) << static_cast<int>(packet[i]) << " ";
            }
            // 计算程序生成的 checksum，并打印包内的 checksum 字节，便于比对
            uint16_t computed_ck = this->get_CRC16_check_sum(const_cast<uint8_t*>(packet.data()), static_cast<uint32_t>(packet.size() - 2), CRC16_INIT);
            uint8_t packet_ck_lo = packet[packet.size() - 2];
            uint8_t packet_ck_hi = packet[packet.size() - 1];
            RCLCPP_WARN(this->get_logger(), "CRC16 check failed, discarding packet. packet_hex=%s", oss.str().c_str());
            RCLCPP_INFO(this->get_logger(), "computed checksum=0x%04x, packet checksum lo=0x%02x hi=0x%02x", (unsigned)computed_ck, (unsigned)packet_ck_lo, (unsigned)packet_ck_hi);
        }
        buffer_.erase(buffer_.begin(), buffer_.begin() + packet_size);
    }
}

uint32_t SerialRxNode::verify_CRC16_check_sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wExpected = 0;
    if ((pchMessage == NULL) || (dwLength <= 2))
    {
        return 0;
    }
    wExpected = this->get_CRC16_check_sum(pchMessage, dwLength - 2, CRC16_INIT);
    // low byte first, high byte second in message
    return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

void SerialRxNode::parse_buffer(const std::vector<uint8_t>& packet){
    nuc_rx_t rx_data;
    // copy into local struct to avoid alignment/aliasing issues
    std::memcpy(&rx_data, packet.data(), std::min(packet.size(), sizeof(nuc_rx_t)));
    for(int i = 0; i < 6; ++i){
        joint_angle_[i] = rx_data.joint_theta[i];
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

uint16_t SerialRxNode::get_CRC16_check_sum(uint8_t *pch_message,uint32_t dw_length,uint16_t wCRC)
{
    uint8_t chData;
    if (pch_message == NULL)
    {
        return 0xFFFF;
    }
    while(dw_length--)
    {
        chData = *pch_message++;
        (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
    }
    return wCRC;
}

