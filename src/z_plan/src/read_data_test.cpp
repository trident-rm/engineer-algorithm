#include <rclcpp/rclcpp.hpp>
#include "usb_data.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto serial_node = std::make_shared<SerialRxNode>();

    std::thread([serial_node]() {
        serial_node->receive_data();
    }).detach();
    
    rclcpp::Rate rate(1.0);
    while (rclcpp::ok()) {
        serial_node->print_joint_angles();
        rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}