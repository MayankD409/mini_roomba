#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class WalkerNode : public rclcpp::Node {
public:
    WalkerNode() : Node("walker_node") {
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&WalkerNode::laserCallback, this, std::placeholders::_1));

        // Initialize the twist message
        twist_msg_.linear.x = 0.2;  // linear velocity
        twist_msg_.angular.z = 0.0; // angular velocity
    }

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Check for obstacles in the front region
        bool obstacle_detected = false;
        for (float range : msg->ranges) {
            if (range < 0.5) { // adjust the threshold based on your robot's dimensions
                obstacle_detected = true;
                break;
            }
        }

        // Adjust robot's behavior based on obstacle detection
        if (obstacle_detected) {
            RCLCPP_INFO(get_logger(), "Obstacle detected. Rotating...");
            twist_msg_.linear.x = 0.0;
            twist_msg_.angular.z = 0.2; // angular velocity for rotation
        } else {
            RCLCPP_INFO(get_logger(), "Moving forward...");
            twist_msg_.linear.x = 0.2;  // linear velocity for forward movement
            twist_msg_.angular.z = 0.0;
        }

        // Publish the twist message
        cmd_vel_pub_->publish(twist_msg_);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    geometry_msgs::msg::Twist twist_msg_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WalkerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
