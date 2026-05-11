/**
 * @file keyboard_input.cpp
 * @brief 把 stdin 单字符转成 /key_input (std_msgs/String) 发布.
 *        必须用 ros2 run 启动 (它会拿到终端 stdin), 不能用 ros2 launch.
 */
#include <termios.h>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("keyboard_input");
    auto pub = node->create_publisher<std_msgs::msg::String>("/key_input", 10);

    termios old_t{};
    tcgetattr(STDIN_FILENO, &old_t);
    termios t = old_t;
    t.c_lflag &= ~(ICANON | ECHO);
    t.c_cc[VMIN] = 1; t.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &t);

    RCLCPP_INFO(node->get_logger(),
                "Keyboard input ready. Press keys (q to quit).");

    char c = 0;
    while (rclcpp::ok()) {
        ssize_t n = read(STDIN_FILENO, &c, 1);
        if (n <= 0) continue;
        std_msgs::msg::String msg;
        msg.data = std::string(1, c);
        pub->publish(msg);
        RCLCPP_INFO(node->get_logger(), "key: '%c' (0x%02x)", c, (unsigned)c);
        if (c == 'q' || c == 'Q') break;
    }
    tcsetattr(STDIN_FILENO, TCSANOW, &old_t);
    rclcpp::shutdown();
    return 0;
}
