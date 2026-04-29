#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <iomanip>
#include <sstream>
#include "tools/usb_data.hpp"

// CRC16查询表和初始值（与usb_data.cpp相同）
uint16_t CRC16_INIT = 0xffff;
const uint16_t wCRC_table[256] = {
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
    0xb5cb, 0xa442, 0x96d9, 0x8750, 0xf3ef, 0xe266, 0xd0fd, 0xc174,
    0x3983, 0x280a, 0x1a91, 0x0b18, 0x7fa7, 0x6e2e, 0x5cb5, 0x4d3c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

// CRC16计算函数（与usb_data.cpp中的get_CRC16_check_sum相同）
uint16_t calculate_CRC16(uint8_t *pch_message, uint32_t dw_length, uint16_t wCRC) {
    uint8_t chData;
    if (pch_message == NULL) {
        return 0xFFFF;
    }
    while(dw_length--) {
        chData = *pch_message++;
        wCRC = ((uint16_t)(wCRC) >> 8) ^ wCRC_table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
    }
    return wCRC;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto serial_node = std::make_shared<SerialRxNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(serial_node);
    
    std::thread executor_thread([&executor]() { 
        executor.spin(); 
    });

    rclcpp::Rate rate(50.0);
    
    size_t counter = 0;
    
    while (rclcpp::ok()) {
        if (counter % 50 == 0) {
            serial_node->print_joint_angles();
        }
        
        // 获取发送数据
        auto currents = serial_node->get_joint_currents();
        auto robo_angles = serial_node->get_robo_arm_angles();
        
        // 打印发送前的原始数据
        if (counter % 50 == 0) {
            std::stringstream ss;
            ss << std::hex << std::setfill('0');
            
            // 打印电流值（float的16进制表示）
            ss << "Currents (hex): ";
            for (const auto& c : currents) {
                uint32_t bits = *reinterpret_cast<const uint32_t*>(&c);
                ss << std::setw(8) << bits << " ";
            }
            RCLCPP_INFO(serial_node->get_logger(), "%s", ss.str().c_str());
            
            // 打印角度值（float的16进制表示）
            ss.str("");
            ss << "RoboArm Angles (hex): ";
            for (const auto& a : robo_angles) {
                uint32_t bits = *reinterpret_cast<const uint32_t*>(&a);
                ss << std::setw(8) << bits << " ";
            }
            RCLCPP_INFO(serial_node->get_logger(), "%s", ss.str().c_str());
            
            // 构建完整的发送数据包并打印16进制
            roboarm_with_controller_tx_t data;
            data.SOF = 0x91;  // EXPECTED_SOF
            
            for (int i = 0; i < 6; ++i) {
                if (i == 3 || i == 4) {
                    data.robo_arm_joint_theta[i] = static_cast<int16_t>(robo_angles[i] * 10000.0f - 31000.0f);
                } else {
                    data.robo_arm_joint_theta[i] = static_cast<int16_t>(robo_angles[i] * 10000.0f);
                }
                data.current[i] = currents[i];
            }
            
            // 计算正确的CRC16校验和（与usb_data.cpp中的方法相同）
            uint8_t* data_ptr_for_crc = reinterpret_cast<uint8_t*>(&data);
            uint32_t data_length = sizeof(roboarm_with_controller_tx_t) - sizeof(uint16_t);
            data.CheckSum = calculate_CRC16(data_ptr_for_crc, data_length, CRC16_INIT);
            
            // 打印完整的二进制数据
            ss.str("");
            ss << std::dec;
            RCLCPP_INFO(serial_node->get_logger(), "========== COMPLETE TX PACKET (HEX) ==========");
            
            uint8_t* data_ptr = reinterpret_cast<uint8_t*>(&data);
            size_t data_size = sizeof(roboarm_with_controller_tx_t);
            
            ss.str("");
            ss << std::hex << std::setfill('0');
            for (size_t i = 0; i < data_size; ++i) {
                ss << std::setw(2) << (int)data_ptr[i];
                if ((i + 1) % 16 == 0) {
                    RCLCPP_INFO(serial_node->get_logger(), "%s", ss.str().c_str());
                    ss.str("");
                    ss << std::hex << std::setfill('0');
                } else {
                    ss << " ";
                }
            }
            if (ss.str().length() > 0) {
                RCLCPP_INFO(serial_node->get_logger(), "%s", ss.str().c_str());
            }
            RCLCPP_INFO(serial_node->get_logger(), "CheckSum: 0x%04x", data.CheckSum);
            RCLCPP_INFO(serial_node->get_logger(), "============================================");
        }
        
        // 使用实际接收到的 robo_arm 数据发送
        serial_node->send_roboarm_with_controller(currents, robo_angles);
        
        counter++;
        rate.sleep();
    }
    
    rclcpp::shutdown();
    executor_thread.join(); 
    return 0;
}