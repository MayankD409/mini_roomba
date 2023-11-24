#include <rosbag2_cpp/writer.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class Walker {
public:
    Walker() : linear_velocity_(0.2), angular_velocity_(0.0) {}

    void update(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

        // Adjust robot's behavior based on obstacle detection
        if (msg->ranges[0] < 0.6) {
            linear_velocity_ = 0.0;
            angular_velocity_ = 0.2; // angular velocity for rotation
        } else {
            linear_velocity_ = 0.1;  // linear velocity for forward movement
            angular_velocity_ = 0.0;
        }
    }

    double getLinearVelocity() const {
        return linear_velocity_;
    }

    double getAngularVelocity() const {
        return angular_velocity_;
    }

private:
    double linear_velocity_;
    double angular_velocity_;
};

class WalkerNode : public rclcpp::Node {
public:
    WalkerNode() : Node("walker_node") {
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&WalkerNode::laserCallback, this, std::placeholders::_1));
        walker_ = std::make_shared<Walker>();
        this->declare_parameter("record_bag", 1);
        writer_ = std::make_unique<rosbag2_cpp::Writer>();
        writer_->open("my_bag");

        // Initialize the twist message
        twist_msg_.linear.x = walker_->getLinearVelocity();
        twist_msg_.angular.z = walker_->getAngularVelocity();
    }

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        walker_->update(msg);

        // Update twist message based on Walker's state
        twist_msg_.linear.x = walker_->getLinearVelocity();
        twist_msg_.angular.z = walker_->getAngularVelocity();

        // Publish the twist message
        cmd_vel_pub_->publish(twist_msg_);
        if (this->get_parameter("record_bag").as_int() == 1) {
            rclcpp::Time time_stamp = this->now();
            writer_->write(twist_msg_, "cmd_vel", time_stamp);
            writer_->write(*msg, "scan", time_stamp);
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    geometry_msgs::msg::Twist twist_msg_;
    std::unique_ptr<rosbag2_cpp::Writer> writer_;
    std::shared_ptr<Walker> walker_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WalkerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
