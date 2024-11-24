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
        : node_(node), currentState_(&movingForwardState_), obstacleDetected_(false), rotateClockwise_(false) {
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
    bool shouldRotateClockwise() const { return rotateClockwise_; }
    void toggleRotationDirection() { rotateClockwise_ = !rotateClockwise_; }

private:
    rclcpp::Node* node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocityPublisher_;
    RobotStateInterface* currentState_;
    bool obstacleDetected_;
    bool rotateClockwise_; // Tracks the current rotation direction

    class MovingForwardState : public RobotStateInterface {
    public:
        RobotStateInterface* transition(Robot& context, bool obstacle_flag) override {
            if (obstacle_flag) {
                // Toggle rotation direction when transitioning to RotatingState
                context.toggleRotationDirection();
                RCLCPP_INFO(context.getNode()->get_logger(),
                            "Transitioning to RotatingState. Next rotation direction: %s",
                            context.shouldRotateClockwise() ? "Clockwise" : "Counterclockwise");
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
            if (!context.obstacleDetected_) {
                // Explicit transition back to MovingForwardState
                context.processInput(false);
                return;
            }

            // Use the current rotation direction
            double angularVelocity = context.shouldRotateClockwise() ? -0.7 : 0.7;
            RCLCPP_INFO(context.getNode()->get_logger(), "Rotating in place. Direction: %s",
                        context.shouldRotateClockwise() ? "Clockwise" : "Counterclockwise");

            context.publishVelocity(0.0, angularVelocity);
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
        const auto& scanData = msg->ranges;
        const float frontMinAngle = -0.35;  // Radians, approximately -11 degrees
        const float frontMaxAngle = 0.35;   // Radians, approximately 11 degrees
        const float angleIncrement = msg->angle_increment;
        const float minObstacleRange = 0.5;  // Distance threshold for obstacle detection

        // Identify indices corresponding to the forward-facing sector
        int startIndex = static_cast<int>((frontMinAngle - msg->angle_min) / angleIncrement);
        int endIndex = static_cast<int>((frontMaxAngle - msg->angle_min) / angleIncrement);

        // Clamp indices to ensure they're within bounds
        startIndex = std::max(0, startIndex);
        endIndex = std::min(static_cast<int>(scanData.size() - 1), endIndex);

        // Check if an obstacle exists in the forward-facing sector
        bool obstacleDetected = false;
        float closestObstacleRange = std::numeric_limits<float>::infinity();

        for (int i = startIndex; i <= endIndex; ++i) {
            if (std::isfinite(scanData[i]) && scanData[i] < minObstacleRange) {
                obstacleDetected = true;
                closestObstacleRange = std::min(closestObstacleRange, scanData[i]);
            }
        }

        // Log the detection status
        if (obstacleDetected) {
            RCLCPP_INFO(this->get_logger(), "Obstacle detected at range: %.2f meters", closestObstacleRange);
        } else {
            RCLCPP_INFO(this->get_logger(), "No obstacle detected in front.");
        }

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
