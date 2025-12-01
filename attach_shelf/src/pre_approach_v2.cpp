#include <chrono>
#include <cmath>
#include <memory>

#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

class PreApproachNodeV2 : public rclcpp::Node {
   public:
    PreApproachNodeV2() : Node("pre_approach_node_v2") {
        // Declare parameters
        this->declare_parameter<double>("obstacle", 0.3);
        this->declare_parameter<int>("degrees", -90);
        this->declare_parameter<bool>("final_approach", false);

        // Get parameters
        obstacle_distance_ = this->get_parameter("obstacle").as_double();
        rotation_degrees_ = this->get_parameter("degrees").as_int();
        final_approach_ = this->get_parameter("final_approach").as_bool();

        // Create subscription to laser scan
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&PreApproachNodeV2::laser_callback, this, _1));

        // Create subscription to odometry
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/diffbot_base_controller/odom", 10, std::bind(&PreApproachNodeV2::odom_callback, this, _1));

        // Create publisher for velocity commands
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/diffbot_base_controller/cmd_vel_unstamped", 10);

        // Create service client
        approach_client_ = this->create_client<attach_shelf::srv::GoToLoading>("/approach_shelf");

        // Initialize state
        state_ = State::MOVING_FORWARD;

        RCLCPP_INFO(this->get_logger(), "Pre-Approach Node V2 Started");
        RCLCPP_INFO(this->get_logger(), "Obstacle distance: %.2f m", obstacle_distance_);
        RCLCPP_INFO(this->get_logger(), "Rotation degrees: %d", rotation_degrees_);
        RCLCPP_INFO(this->get_logger(), "Final approach: %s", final_approach_ ? "true" : "false");

        // Create timer for control loop
        timer_ = this->create_wall_timer(
            100ms, std::bind(&PreApproachNodeV2::control_loop, this));
    }

   private:
    enum class State {
        MOVING_FORWARD,
        ROTATING,
        CALLING_SERVICE,
        COMPLETED
    };

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Get the front distance (center of laser scan)
        int center_idx = msg->ranges.size() / 2;

        // Average a few readings around the center for robustness
        double min_distance = std::numeric_limits<double>::max();
        int range = 10;  // Check +/- 10 readings around center

        for (int i = center_idx - range; i <= center_idx + range; i++) {
            if (i >= 0 && i < static_cast<int>(msg->ranges.size())) {
                if (std::isfinite(msg->ranges[i]) && msg->ranges[i] > 0.0) {
                    min_distance = std::min(min_distance, static_cast<double>(msg->ranges[i]));
                }
            }
        }

        front_distance_ = min_distance;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Extract yaw from quaternion
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);

        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        current_yaw_ = yaw;
    }

    void call_approach_service() {
        RCLCPP_INFO(this->get_logger(), "Calling approach_shelf service...");

        // Wait for service to be available
        while (!approach_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service");
                state_ = State::COMPLETED;
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for /approach_shelf service...");
        }

        // Create request
        auto request = std::make_shared<attach_shelf::srv::GoToLoading::Request>();
        request->attach_to_shelf = final_approach_;

        // Send async request with callback
        auto response_callback = [this](rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedFuture future) {
            auto result = future.get();
            RCLCPP_INFO(this->get_logger(), "Service call completed: %s",
                        result->complete ? "SUCCESS" : "FAILED");
            state_ = State::COMPLETED;
        };

        approach_client_->async_send_request(request, response_callback);
        RCLCPP_INFO(this->get_logger(), "Service request sent, waiting for response...");
    }

    void control_loop() {
        auto twist_msg = geometry_msgs::msg::Twist();

        switch (state_) {
            case State::MOVING_FORWARD: {
                // Check if we've reached the obstacle
                if (front_distance_ <= obstacle_distance_) {
                    RCLCPP_INFO(this->get_logger(), "Obstacle detected at %.2f m. Stopping.", front_distance_);

                    // Stop the robot
                    vel_pub_->publish(twist_msg);

                    // Transition to rotating state
                    state_ = State::ROTATING;

                    // Store initial yaw for rotation tracking
                    initial_yaw_ = current_yaw_;
                    target_yaw_ = initial_yaw_ + (rotation_degrees_ * M_PI / 180.0);

                    // Normalize target yaw to [-pi, pi]
                    while (target_yaw_ > M_PI) target_yaw_ -= 2.0 * M_PI;
                    while (target_yaw_ < -M_PI) target_yaw_ += 2.0 * M_PI;

                    RCLCPP_INFO(this->get_logger(),
                                "Starting rotation of %d degrees (from %.2f to %.2f rad)",
                                rotation_degrees_, initial_yaw_, target_yaw_);
                    break;
                }

                // Move forward
                twist_msg.linear.x = linear_velocity_;
                twist_msg.angular.z = 0.0;
                vel_pub_->publish(twist_msg);
                break;
            }

            case State::ROTATING: {
                // Calculate angle difference (handling wrap-around)
                double angle_diff = target_yaw_ - current_yaw_;

                // Normalize angle difference to [-pi, pi]
                while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
                while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

                // Check if rotation is complete (within tolerance)
                if (std::abs(angle_diff) < angle_tolerance_) {
                    RCLCPP_INFO(this->get_logger(),
                                "Rotation completed. Final yaw: %.2f rad", current_yaw_);

                    // Stop the robot
                    vel_pub_->publish(twist_msg);

                    // Call service if needed
                    state_ = State::CALLING_SERVICE;
                    call_approach_service();
                    break;
                }

                // Continue rotating (proportional control for smoother stopping)
                twist_msg.linear.x = 0.0;

                // Use proportional control near target, full speed otherwise
                if (std::abs(angle_diff) < 0.3) {            // Within ~17 degrees
                    twist_msg.angular.z = angle_diff * 2.0;  // Proportional gain
                } else {
                    twist_msg.angular.z = (angle_diff > 0) ? angular_velocity_ : -angular_velocity_;
                }

                vel_pub_->publish(twist_msg);
                break;
            }

            case State::CALLING_SERVICE: {
                // Service is being called in another thread
                break;
            }

            case State::COMPLETED: {
                // Stop the timer to prevent further callbacks
                if (timer_) {
                    timer_->cancel();
                    timer_.reset();
                }

                RCLCPP_INFO(this->get_logger(), "Pre-approach V2 completed. Shutting down node.");
                rclcpp::shutdown();
                break;
            }
        }
    }

    // ROS2 components
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedPtr approach_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters
    double obstacle_distance_;
    int rotation_degrees_;
    bool final_approach_;

    // State variables
    State state_;
    double front_distance_ = std::numeric_limits<double>::max();
    double current_yaw_ = 0.0;
    double initial_yaw_ = 0.0;
    double target_yaw_ = 0.0;

    // Control parameters
    const double linear_velocity_ = 0.5;
    const double angular_velocity_ = 0.8;
    const double angle_tolerance_ = 0.02;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PreApproachNodeV2>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
