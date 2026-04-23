#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <Eigen/Dense>
#include <iomanip>
#include <sstream>
#include <cmath>

class EndEffectorPositionNode : public rclcpp::Node
{
public:
    EndEffectorPositionNode() : Node("end_effector_position")
    {
        // Subscribe to joint states
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&EndEffectorPositionNode::jointStateCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "End Effector Position Node started");
        RCLCPP_INFO(this->get_logger(), "Using Forward Kinematics with DH parameters");
        RCLCPP_INFO(this->get_logger(), "Waiting for joint states...");
    }

private:
    // DH parameters for each joint (from URDF)
    struct DHParams {
        double a;      // link length
        double d;      // link offset
        double alpha;  // link twist
    };
    
    // Create DH transformation matrix
    Eigen::Matrix4d dhTransform(double a, double d, double alpha, double theta)
    {
        Eigen::Matrix4d T;
        double ca = cos(alpha);
        double sa = sin(alpha);
        double ct = cos(theta);
        double st = sin(theta);
        
        T << ct,    -st*ca,   st*sa,   a*ct,
             st,     ct*ca,  -ct*sa,   a*st,
              0,        sa,      ca,      d,
              0,         0,       0,      1;
        
        return T;
    }
    
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.size() < 6) {
            RCLCPP_WARN(this->get_logger(), "Not enough joint positions received");
            return;
        }
        
        // Extract joint angles (theta values)
        std::vector<double> theta(6);
        for (size_t i = 0; i < 6; i++) {
            theta[i] = msg->position[i];
        }
        
        // DH parameters from URDF (a, d, alpha)
        std::vector<DHParams> dh = {
            {0.0,   0.1,  0.0},           // joint1
            {0.1,   0.3,  M_PI/2},        // joint2
            {0.3,   0.08, 0.0},           // joint3
            {0.25,  0.15, -M_PI/2},       // joint4
            {0.0,   0.1,  -M_PI/2},       // joint5
            {0.15,  0.15, M_PI/2}         // joint6
        };
        
        // Calculate forward kinematics by multiplying transformation matrices
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        
        for (size_t i = 0; i < 6; i++) {
            T = T * dhTransform(dh[i].a, dh[i].d, dh[i].alpha, theta[i]);
        }
        
        // Add the fixed transform from link6 to end_effector (z offset of 0.4)
        Eigen::Matrix4d T_end = Eigen::Matrix4d::Identity();
        T_end(2, 3) = 0.4;  // z offset
        T = T * T_end;
        
        // Extract position
        double x = T(0, 3);
        double y = T(1, 3);
        double z = T(2, 3);
        
        // Extract rotation and convert to roll, pitch, yaw
        double roll = atan2(T(2, 1), T(2, 2));
        double pitch = atan2(-T(2, 0), sqrt(T(2, 1)*T(2, 1) + T(2, 2)*T(2, 2)));
        double yaw = atan2(T(1, 0), T(0, 0));
        
        // Print the position and orientation
        std::stringstream ss;
        ss << std::fixed << std::setprecision(4);
        ss << "\n--- End Effector Position (FK) ---\n";
        ss << "Position (x, y, z): (" << x << ", " << y << ", " << z << ") meters\n";
        ss << "Joint angles: [";
        for (size_t i = 0; i < theta.size(); i++) {
            ss << std::setprecision(3) << theta[i];
            if (i < theta.size() - 1) ss << ", ";
        }
        ss << "] rad\n";
        ss << std::setprecision(4);
        ss << "Orientation (roll, pitch, yaw): (" << roll << ", " << pitch << ", " << yaw << ") radians\n";
        ss << std::setprecision(2);
        ss << "Orientation (degrees): (" 
           << (roll * 180.0 / M_PI) << "°, " 
           << (pitch * 180.0 / M_PI) << "°, " 
           << (yaw * 180.0 / M_PI) << "°)";
        
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    }
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EndEffectorPositionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
