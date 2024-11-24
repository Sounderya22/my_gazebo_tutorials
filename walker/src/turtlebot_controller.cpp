// Copyright 2024 Sounderya Varagur Venugopal
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file turtlebot_controller.cpp
 * @brief ROS 2 Node to control a TurtleBot using a state design pattern for obstacle avoidance.
 *
 * This program implements a simple state machine with two states: MovingForward and Rotating.
 * The robot transitions between these states based on the presence of obstacles detected
 * using LaserScan data.
 *
 * @author Sounderya Varagur Venugopal
 * @date 2024
 */

#include <memory>
#include <vector>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

/**
 * @class RobotStateInterface
 * @brief Abstract base class for defining a common interface for all concrete state classes.
 */
class RobotStateInterface {
 public:
  virtual ~RobotStateInterface() = default;

  /**
   * @brief Transition logic based on obstacle presence.
   * @param context The robot context to manage state transitions.
   * @param obstacle_flag Whether an obstacle is detected.
   * @return The next state for the robot.
   */
  // cpplint suppression for non-const reference:
  // NOLINTNEXTLINE(runtime/references)
  virtual RobotStateInterface* Transition(class Robot& context, bool obstacle_flag) = 0;

  /**
   * @brief Execute the behavior associated with the current state.
   * @param context The robot context for performing actions.
   */
  // cpplint suppression for non-const reference:
  // NOLINTNEXTLINE(runtime/references)
  virtual void Execute(class Robot& context) = 0;
};

/**
 * @class Robot
 * @brief Context class for defining the robot .
 */
class Robot {
 public:
  explicit Robot(rclcpp::Node* node)
      : node_(node),
        current_state_(&moving_forward_state_),
        obstacle_detected_(false),
        rotate_counterclockwise_(false) {
    velocity_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    RCLCPP_INFO(node_->get_logger(), "Robot instance initialized.");
  }

  /**
   * @brief Processes input to determine the next state based on obstacle detection.
   * @param obstacle_flag Whether an obstacle is detected.
   */
  void ProcessInput(bool obstacle_flag) {
    obstacle_detected_ = obstacle_flag;
    RobotStateInterface* new_state = current_state_->Transition(*this, obstacle_detected_);
    if (new_state != current_state_) {
      current_state_ = new_state;
      RCLCPP_INFO(node_->get_logger(), "State transitioned.");
    }
  }

  /**
   * @brief Executes the behavior associated with the current state.
   */
  void ExecuteCurrentState() { current_state_->Execute(*this); }

  /**
   * @brief Publishes velocity commands.
   * @param linear Linear velocity.
   * @param angular Angular velocity.
   */
  void PublishVelocity(double linear, double angular) {
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = linear;
    twist.angular.z = angular;
    velocity_publisher_->publish(twist);
  }

  rclcpp::Node* GetNode() const { return node_; }
  bool ShouldRotateClockwise() const { return rotate_counterclockwise_; }
  void ToggleRotationDirection() { rotate_counterclockwise_ = !rotate_counterclockwise_; }

 private:
  rclcpp::Node* node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  RobotStateInterface* current_state_;
  bool obstacle_detected_;
  bool rotate_counterclockwise_;

  /**
   * @class MovingForwardState
   * @brief State where the robot moves forward.
   */
  class MovingForwardState : public RobotStateInterface {
   public:
    RobotStateInterface* Transition(Robot& context, bool obstacle_flag) override {
      if (obstacle_flag) {
        context.ToggleRotationDirection();
        RCLCPP_INFO(context.GetNode()->get_logger(),
                    "Transitioning to RotatingState. Next rotation direction: %s",
                    context.ShouldRotateClockwise() ? "Clockwise" : "Counterclockwise");
        return &context.rotating_state_;
      }
      return this;
    }

    void Execute(Robot& context) override {
      RCLCPP_INFO(context.GetNode()->get_logger(), "Moving Forward.");
      context.PublishVelocity(0.2, 0.0);
    }
  };

  /**
   * @class RotatingState
   * @brief State where the robot rotates in place to avoid obstacles.
   */
  class RotatingState : public RobotStateInterface {
   public:
    RobotStateInterface* Transition(Robot& context, bool obstacle_flag) override {
      if (!obstacle_flag) {
        RCLCPP_INFO(context.GetNode()->get_logger(), "Obstacle cleared. Transitioning to MovingForwardState.");
        return &context.moving_forward_state_;
      }
      return this;
    }

    void Execute(Robot& context) override {
      if (!context.obstacle_detected_) {
        context.ProcessInput(false);
        return;
      }

      double angular_velocity = context.ShouldRotateClockwise() ? -0.7 : 0.7;
      RCLCPP_INFO(context.GetNode()->get_logger(), "Rotating in place. Direction: %s",
                  context.ShouldRotateClockwise() ? "Clockwise" : "Counterclockwise");
      context.PublishVelocity(0.0, angular_velocity);
    }
  };

  MovingForwardState moving_forward_state_;
  RotatingState rotating_state_;
};

/**
 * @class TurtleBotController
 * @brief ROS 2 Node to manage the TurtleBot's obstacle avoidance behavior.
 */
class TurtleBotController : public rclcpp::Node {
 public:
  TurtleBotController()
      : Node("turtlebot_controller"), robot_(std::make_shared<Robot>(this)) {
    RCLCPP_INFO(this->get_logger(), "TurtleBotController initialized.");
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&TurtleBotController::ScanCallback, this, std::placeholders::_1));
  }

 private:
  void ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    const auto& scan_data = msg->ranges;
    const float front_min_angle = -0.35;  // Radians
    const float front_max_angle = 0.35;  // Radians
    const float angle_increment = msg->angle_increment;
    const float min_obstacle_range = 0.5;  // Meters

    int start_index = static_cast<int>((front_min_angle - msg->angle_min) / angle_increment);
    int end_index = static_cast<int>((front_max_angle - msg->angle_min) / angle_increment);

    start_index = std::max(0, start_index);
    end_index = std::min(static_cast<int>(scan_data.size() - 1), end_index);

    bool obstacle_detected = false;
    float closest_obstacle_range = std::numeric_limits<float>::infinity();

    for (int i = start_index; i <= end_index; ++i) {
      if (std::isfinite(scan_data[i]) && scan_data[i] < min_obstacle_range) {
        obstacle_detected = true;
        closest_obstacle_range = std::min(closest_obstacle_range, scan_data[i]);
      }
    }

    if (obstacle_detected) {
      RCLCPP_INFO(this->get_logger(), "Obstacle detected at range: %.2f meters", closest_obstacle_range);
    } else {
      RCLCPP_INFO(this->get_logger(), "No obstacle detected in front.");
    }

    robot_->ProcessInput(obstacle_detected);
    robot_->ExecuteCurrentState();
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
  std::shared_ptr<Robot> robot_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TurtleBotController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
