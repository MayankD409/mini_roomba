/**
Copyright © 2023 <copyright holders>

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the “Software”), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/

/**
 * @file walker_node.cpp
 * @author Mayank Deshpande
 * @brief Implements a ROS2 node for a simple walker algorithm with rosbag
 * recording.
 * @version 0.1
 * @date 2023-11-23
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <rosbag2_cpp/writer.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

/**
 * @class Walker
 * @brief Represents the state and behavior of a walker robot.
 */
class Walker {
 public:
  /**
   * @brief Constructor for the Walker class.
   */
  Walker() : linear_velocity_(0.2), angular_velocity_(0.0) {}

  /**
   * @brief Update the walker's behavior based on laser scan data.
   * @param msg Laser scan data.
   */
  void update(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Adjust robot's behavior based on obstacle detection
    if (msg->ranges[0] < 0.6) {
      linear_velocity_ = 0.0;
      angular_velocity_ = 0.2;  // angular velocity for rotation
    } else {
      linear_velocity_ = 0.1;  // linear velocity for forward movement
      angular_velocity_ = 0.0;
    }
  }

  /**
   * @brief Get the linear velocity of the walker.
   * @return Linear velocity.
   */
  double getLinearVelocity() const { return linear_velocity_; }

  /**
   * @brief Get the angular velocity of the walker.
   * @return Angular velocity.
   */
  double getAngularVelocity() const { return angular_velocity_; }

 private:
  double linear_velocity_;   ///< Linear velocity of the walker.
  double angular_velocity_;  ///< Angular velocity of the walker.
};

/**
 * @class WalkerNode
 * @brief ROS2 node for the walker algorithm with rosbag recording.
 */
class WalkerNode : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for the WalkerNode class.
   */
  WalkerNode() : Node("walker_node") {
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&WalkerNode::laserCallback, this, std::placeholders::_1));
    walker_ = std::make_shared<Walker>();
    this->declare_parameter("record_bag", 1);
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open("my_bag");

    // Initialize the twist message
    twist_msg_.linear.x = walker_->getLinearVelocity();
    twist_msg_.angular.z = walker_->getAngularVelocity();
  }

 private:
  /**
   * @brief Callback function for laser scan data.
   * @param msg Laser scan data.
   */
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    walker_->update(msg);

    // Update twist message based on Walker's state
    twist_msg_.linear.x = walker_->getLinearVelocity();
    twist_msg_.angular.z = walker_->getAngularVelocity();

    // Publish the twist message
    cmd_vel_pub_->publish(twist_msg_);

    // Record data in rosbag if the "record_bag" parameter is set to 1
    if (this->get_parameter("record_bag").as_int() == 1) {
      rclcpp::Time time_stamp = this->now();
      writer_->write(twist_msg_, "cmd_vel", time_stamp);
      writer_->write(*msg, "scan", time_stamp);
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
      cmd_vel_pub_;  ///< Publisher for cmd_vel topic.
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_sub_;                        ///< Subscriber for scan topic.
  geometry_msgs::msg::Twist twist_msg_;  ///< Twist message for robot control.
  std::unique_ptr<rosbag2_cpp::Writer> writer_;  ///< ROS2 bag file writer.
  std::shared_ptr<Walker> walker_;  ///< Instance of the Walker class.
};

/**
 * @brief Main function for the walker_node executable.
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return Exit code.
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WalkerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
