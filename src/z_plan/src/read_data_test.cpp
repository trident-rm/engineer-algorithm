#include <rclcpp/rclcpp.hpp>
#include "usb_data.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto serial_node = std::make_shared<SerialRxNode>();

    std::thread([serial_node]() {
        serial_node->receive_data();
    }).detach();

    rclcpp::Rate rate(50.0);
    
    size_t counter = 0;
    
    while (rclcpp::ok()) {
        if (counter % 50 == 0) {
            serial_node->print_joint_angles();
        }
        
        // 使用实际接收到的 robo_arm 数据发送
        serial_node->send_roboarm_with_controller(
            serial_node->get_joint_currents(), 
            serial_node->get_robo_arm_angles()
        );
        
        counter++;
        rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}