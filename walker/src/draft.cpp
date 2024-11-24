#include <memory>
#include <vector>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

// Abstract class: State Interface
class RobotStateInterface {
public:
    virtual ~RobotStateInterface() = default;

    virtual RobotStateInterface* transition(class Robot& context, bool obstacle_flag) = 0;
    virtual void execute(class Robot& context) = 0;
};

// Context class to manage the robot states
class Robot {
public:
    explicit Robot(rclcpp::Node* node)
        : node_(node), currentState_(&movingForwardState_), obstacleDetected_(false) {
        velocityPublisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        RCLCPP_INFO(node_->get_logger(), "Robot instance initialized.");
    }

    void processInput(bool obstacle_flag) {
        obstacleDetected_ = obstacle_flag;
        RobotStateInterface* newState = currentState_->transition(*this, obstacleDetected_);
        if (newState != currentState_) {
            currentState_ = newState;
            RCLCPP_INFO(node_->get_logger(), "State transitioned.");
        }
    }

    void executeCurrentState() {
        currentState_->execute(*this);
    }

    void publishVelocity(double linear, double angular) {
        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = linear;
        twist.angular.z = angular;
        velocityPublisher_->publish(twist);
    }

    rclcpp::Node* getNode() const { return node_; }

private:
    rclcpp::Node* node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocityPublisher_;
    RobotStateInterface* currentState_;
    bool obstacleDetected_;

    class MovingForwardState : public RobotStateInterface {
    public:
        RobotStateInterface* transition(Robot& context, bool obstacle_flag) override {
            if (obstacle_flag) {
                RCLCPP_INFO(context.getNode()->get_logger(), "Transitioning to RotatingState due to obstacle.");
                return &context.rotatingState_;
            }
            return this;
        }

        void execute(Robot& context) override {
            RCLCPP_INFO(context.getNode()->get_logger(), "Moving Forward.");
            context.publishVelocity(0.2, 0.0);
        }
    };

    class RotatingState : public RobotStateInterface {
    public:
        RobotStateInterface* transition(Robot& context, bool obstacle_flag) override {
            if (!obstacle_flag) {
                RCLCPP_INFO(context.getNode()->get_logger(), "Obstacle cleared. Transitioning to MovingForwardState.");
                return &context.movingForwardState_;
            }
            return this;
        }

        void execute(Robot& context) override {
            RCLCPP_INFO(context.getNode()->get_logger(), "Rotating in place to avoid obstacle.");
            context.publishVelocity(0.0, 0.5);
        }
    };

    MovingForwardState movingForwardState_;
    RotatingState rotatingState_;
};

// ROS2 Node to control the robot
class TurtleBotController : public rclcpp::Node {
public:
    TurtleBotController()
        : Node("turtlebot_controller"), robot_(std::make_shared<Robot>(this)) {
        RCLCPP_INFO(this->get_logger(), "TurtleBotController initialized.");
        scanSubscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&TurtleBotController::scanCallback, this, std::placeholders::_1));
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Ensure the ranges array is not empty
        if (msg->ranges.empty()) {
            RCLCPP_WARN(this->get_logger(), "LaserScan ranges array is empty. Skipping scan processing.");
            return;
        }

        // Define parameters for obstacle detection
        const float OBSTACLE_DISTANCE_THRESHOLD = 0.5; // meters
        const int WINDOW_SIZE = 30; // Number of indices around the center to check

        // Get the center index of the laser scan ranges
        int centerIndex = msg->ranges.size() / 2;

        // Check a window around the center index for obstacles
        bool obstacleDetected = false;
        for (int i = -WINDOW_SIZE; i <= WINDOW_SIZE; ++i) {
            int index = centerIndex + i;

            // Ensure index is within bounds and check range validity
            if (index >= 0 && index < static_cast<int>(msg->ranges.size())) {
                float range = msg->ranges[index];
                if (std::isfinite(range) && range < OBSTACLE_DISTANCE_THRESHOLD) {
                    obstacleDetected = true;
                    break;
                }
            }
        }

        // Log information about detection
        RCLCPP_INFO(this->get_logger(), "Obstacle detected: %s", obstacleDetected ? "YES" : "NO");

        // Update the robot's state machine
        robot_->processInput(obstacleDetected);
        robot_->executeCurrentState();
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanSubscriber_;
    std::shared_ptr<Robot> robot_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleBotController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}