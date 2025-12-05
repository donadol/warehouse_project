#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class ApproachServiceServer : public rclcpp::Node {
   public:
    ApproachServiceServer() : Node("approach_service_server_node") {
        // Declare parameters with defaults for simulation
        this->declare_parameter<std::string>("scan_topic", "/scan");
        this->declare_parameter<std::string>("odom_topic", "/diffbot_base_controller/odom");
        this->declare_parameter<std::string>("cmd_vel_topic", "/diffbot_base_controller/cmd_vel_unstamped");
        this->declare_parameter<std::string>("global_frame", "map");
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("base_frame", "robot_base_link");
        this->declare_parameter<double>("intensity_thres_fact", 0.0);
        this->declare_parameter<double>("intensity_threshold", 8000.0);
        this->declare_parameter<double>("align_ahead_dist", 0.30);
        this->declare_parameter<double>("align_cart_center_dist", 0.48);

        // Get parameter values
        std::string scan_topic = this->get_parameter("scan_topic").as_string();
        std::string odom_topic = this->get_parameter("odom_topic").as_string();
        std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
        global_frame_ = this->get_parameter("global_frame").as_string();
        odom_frame_ = this->get_parameter("odom_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        intensity_thres_fact_ = this->get_parameter("intensity_thres_fact").as_double();
        intensity_threshold_ = this->get_parameter("intensity_threshold").as_double();
        align_ahead_dist_ = this->get_parameter("align_ahead_dist").as_double();
        align_cart_center_dist_ = this->get_parameter("align_cart_center_dist").as_double();

        RCLCPP_INFO(this->get_logger(), "Using topics:");
        RCLCPP_INFO(this->get_logger(), "  scan: %s", scan_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  odom: %s", odom_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  cmd_vel: %s", cmd_vel_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Using frames:");
        RCLCPP_INFO(this->get_logger(), "  global_frame: %s", global_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  odom_frame: %s", odom_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  base_frame: %s", base_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Intensity detection:");
        if (intensity_thres_fact_ > 0.0) {
            RCLCPP_INFO(this->get_logger(), "  Using adaptive threshold: %.0f%% of max intensity", intensity_thres_fact_ * 100.0);
        } else {
            RCLCPP_INFO(this->get_logger(), "  Using absolute threshold: %.1f", intensity_threshold_);
        }
        RCLCPP_INFO(this->get_logger(), "Cart alignment:");
        RCLCPP_INFO(this->get_logger(), "  align_ahead_dist: %.2fm", align_ahead_dist_);
        RCLCPP_INFO(this->get_logger(), "  align_cart_center_dist: %.2fm", align_cart_center_dist_);

        // Create callback groups for concurrent execution
        service_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // Create service with its own callback group
        service_ = this->create_service<attach_shelf::srv::GoToLoading>(
            "/approach_shelf",
            std::bind(&ApproachServiceServer::approach_callback, this, _1, _2),
            rmw_qos_profile_services_default,
            service_callback_group_);

        // Create subscriptions
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic, 10, std::bind(&ApproachServiceServer::laser_callback, this, _1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10, std::bind(&ApproachServiceServer::odom_callback, this, _1));

        // Create publishers
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            cmd_vel_topic, 10);

        elevator_up_pub_ = this->create_publisher<std_msgs::msg::String>("/elevator_up", 10);

        // Create static TF broadcaster
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // Create TF buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Create timer for control loop with its own callback group
        timer_ = this->create_wall_timer(
            100ms, std::bind(&ApproachServiceServer::control_loop, this), timer_callback_group_);

        RCLCPP_INFO(this->get_logger(), "Approach Service Server Started");
    }

   private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        last_laser_scan_ = msg;
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
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
    }

    void approach_callback(
        const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
        std::shared_ptr<attach_shelf::srv::GoToLoading::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Approach service called with attach_to_shelf=%s",
                    request->attach_to_shelf ? "true" : "false");

        // Detect shelf legs
        if (!last_laser_scan_) {
            RCLCPP_ERROR(this->get_logger(), "No laser scan data available");
            response->complete = false;
            return;
        }

        auto legs = detect_shelf_legs();

        if (legs.size() < 2) {
            RCLCPP_ERROR(this->get_logger(), "Could not detect both shelf legs (found %zu)", legs.size());
            response->complete = false;
            return;
        }

        // Publish cart_frame TF
        publish_cart_frame(legs);

        // Give TF time to propagate through the system
        RCLCPP_INFO(this->get_logger(), "Waiting for cart_frame TF to propagate...");
        rclcpp::sleep_for(std::chrono::milliseconds(500));

        if (!request->attach_to_shelf) {
            // Only publish TF, don't approach
            response->complete = true;
            RCLCPP_INFO(this->get_logger(), "Published cart_frame TF without approaching");
            return;
        }

        // Verify cart_frame is available in TF tree
        if (!tf_buffer_->canTransform("robot_base_link", "cart_frame", tf2::TimePointZero, std::chrono::seconds(3))) {
            RCLCPP_ERROR(this->get_logger(), "cart_frame TF not available after waiting");
            response->complete = false;
            return;
        }

        // Perform final approach
        service_active_ = true;
        attach_to_shelf_ = true;
        state_ = State::MOVING_TO_CART;

        RCLCPP_INFO(this->get_logger(), "Starting approach to shelf");

        // Wait for approach to complete
        while (service_active_ && rclcpp::ok()) {
            std::this_thread::sleep_for(100ms);
        }

        RCLCPP_INFO(this->get_logger(), "Approach to shelf %s",
                    approach_successful_ ? "successful" : "failed");

        response->complete = approach_successful_;
        return;
    }

    struct LegDetection {
        int index;
        double angle;
        double range;
    };

    std::vector<LegDetection> detect_shelf_legs() {
        std::vector<LegDetection> legs;

        if (!last_laser_scan_) {
            return legs;
        }

        const int MIN_CONSECUTIVE = 3;  // Minimum consecutive readings to consider a leg

        // Find max intensity
        double max_intensity = 0.0;
        int max_intensity_idx = -1;

        for (size_t i = 0; i < last_laser_scan_->intensities.size(); i++) {
            if (last_laser_scan_->intensities[i] > max_intensity) {
                max_intensity = last_laser_scan_->intensities[i];
                max_intensity_idx = i;
            }
        }

        // Calculate threshold (adaptive or absolute)
        double threshold;
        if (intensity_thres_fact_ > 0.0) {
            // Use adaptive threshold as percentage of max intensity
            threshold = max_intensity * intensity_thres_fact_;
            RCLCPP_INFO(this->get_logger(), "Using ADAPTIVE threshold: %.1f (%.0f%% of max %.1f)",
                        threshold, intensity_thres_fact_ * 100.0, max_intensity);
        } else {
            // Use absolute threshold
            threshold = intensity_threshold_;
            RCLCPP_INFO(this->get_logger(), "Using ABSOLUTE threshold: %.1f (max intensity: %.1f)",
                        threshold, max_intensity);
        }

        // Count readings above threshold
        int count_above_threshold = 0;
        for (size_t i = 0; i < last_laser_scan_->intensities.size(); i++) {
            if (last_laser_scan_->intensities[i] >= threshold) {
                count_above_threshold++;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Scan stats: total_points=%zu, max_intensity=%.1f at idx=%d, above_threshold=%d",
                    last_laser_scan_->intensities.size(), max_intensity, max_intensity_idx, count_above_threshold);

        std::vector<int> high_intensity_indices;

        // Find all indices with high intensity
        for (size_t i = 0; i < last_laser_scan_->intensities.size(); i++) {
            if (last_laser_scan_->intensities[i] >= threshold) {
                high_intensity_indices.push_back(i);
            }
        }

        if (high_intensity_indices.empty()) {
            RCLCPP_WARN(this->get_logger(), "No high intensity readings found! Max intensity was %.1f, threshold is %.1f",
                        max_intensity, threshold);
            RCLCPP_INFO(this->get_logger(), "Detected %zu shelf legs", legs.size());
            return legs;
        }

        RCLCPP_INFO(this->get_logger(), "Found %zu high intensity readings", high_intensity_indices.size());

        // Group consecutive indices into legs
        std::vector<std::vector<int>> leg_groups;
        std::vector<int> current_group = {high_intensity_indices[0]};

        for (size_t i = 1; i < high_intensity_indices.size(); i++) {
            if (high_intensity_indices[i] - high_intensity_indices[i - 1] <= 2) {
                current_group.push_back(high_intensity_indices[i]);
                continue;
            }

            // End of a group
            if (current_group.size() >= static_cast<size_t>(MIN_CONSECUTIVE)) {
                leg_groups.push_back(current_group);
                RCLCPP_INFO(this->get_logger(), "Found leg group: size=%zu, indices=[%d-%d]",
                           current_group.size(), current_group.front(), current_group.back());
            } else {
                RCLCPP_DEBUG(this->get_logger(), "Rejected small group: size=%zu (min=%d)",
                            current_group.size(), MIN_CONSECUTIVE);
            }
            current_group = {high_intensity_indices[i]};
        }

        // Check last group
        if (current_group.size() >= static_cast<size_t>(MIN_CONSECUTIVE)) {
            leg_groups.push_back(current_group);
            RCLCPP_INFO(this->get_logger(), "Found leg group: size=%zu, indices=[%d-%d]",
                       current_group.size(), current_group.front(), current_group.back());
        } else {
            RCLCPP_DEBUG(this->get_logger(), "Rejected last small group: size=%zu (min=%d)",
                        current_group.size(), MIN_CONSECUTIVE);
        }

        RCLCPP_INFO(this->get_logger(), "Total leg groups found: %zu", leg_groups.size());

        // Convert groups to leg detections (use center of each group)
        for (size_t i = 0; i < leg_groups.size(); i++) {
            const auto& group = leg_groups[i];
            int center_idx = group[group.size() / 2];
            LegDetection leg;
            leg.index = center_idx;
            leg.angle = last_laser_scan_->angle_min + center_idx * last_laser_scan_->angle_increment;
            leg.range = last_laser_scan_->ranges[center_idx];
            legs.push_back(leg);

            RCLCPP_INFO(this->get_logger(), "Leg %zu: center_idx=%d, angle=%.3f rad, range=%.3f m, intensity=%.1f",
                       i, center_idx, leg.angle, leg.range, last_laser_scan_->intensities[center_idx]);
        }

        RCLCPP_INFO(this->get_logger(), "Detected %zu shelf legs", legs.size());
        return legs;
    }

    void publish_cart_frame(const std::vector<LegDetection>& legs) {
        if (legs.size() < 2) {
            return;
        }

        // Calculate positions between the two detected legs in laser frame
        double angle1 = legs[0].angle;
        double angle2 = legs[1].angle;
        double range1 = legs[0].range;
        double range2 = legs[1].range;

        // Convert to Cartesian coordinates (laser frame)
        double x1 = range1 * cos(angle1);
        double y1 = range1 * sin(angle1);
        double x2 = range2 * cos(angle2);
        double y2 = range2 * sin(angle2);

        // Calculate entry point (midpoint) in laser frame
        double entry_x_laser = (x1 + x2) / 2.0;
        double entry_y_laser = (y1 + y2) / 2.0;

        // Calculate perpendicular vector (pointing INTO the shelf)
        double dx = x2 - x1;
        double dy = y2 - y1;
        double length = std::sqrt(dx * dx + dy * dy);
        double perp_x = -dy / length;  // Perpendicular to shelf (pointing inward)
        double perp_y = dx / length;

        RCLCPP_INFO(this->get_logger(), "=== Cart Frame Geometry ===");
        RCLCPP_INFO(this->get_logger(), "Leg 1: (%.2f, %.2f)", x1, y1);
        RCLCPP_INFO(this->get_logger(), "Leg 2: (%.2f, %.2f)", x2, y2);
        RCLCPP_INFO(this->get_logger(), "Entry point: (%.2f, %.2f)", entry_x_laser, entry_y_laser);
        RCLCPP_INFO(this->get_logger(), "Perpendicular vector: (%.3f, %.3f)", perp_x, perp_y);

        // Calculate all three frame positions in laser frame
        // 1. Align frame: ahead of cart for alignment
        double align_x_laser = entry_x_laser + align_ahead_dist_ * perp_x;
        double align_y_laser = entry_y_laser + align_ahead_dist_ * perp_y;

        // 2. Entry frame: between the legs
        // (already calculated as entry_x_laser, entry_y_laser)

        // 3. Center frame: inside the cart under shelf
        double center_x_laser = entry_x_laser - align_cart_center_dist_ * perp_x;
        double center_y_laser = entry_y_laser - align_cart_center_dist_ * perp_y;

        RCLCPP_INFO(this->get_logger(), "Align frame (%.2fm ahead): (%.2f, %.2f)",
                    align_ahead_dist_, align_x_laser, align_y_laser);
        RCLCPP_INFO(this->get_logger(), "Center frame (%.2fm inside): (%.2f, %.2f)",
                    align_cart_center_dist_, center_x_laser, center_y_laser);

        // Calculate orientation (facing into shelf, perpendicular direction)
        double yaw = std::atan2(perp_y, perp_x);
        tf2::Quaternion orientation;
        orientation.setRPY(0, 0, yaw);

        RCLCPP_INFO(this->get_logger(), "Cart orientation: yaw=%.2f rad (%.1f deg)",
                    yaw, yaw * 180.0 / M_PI);

        // Transform all three points to global frame
        std::string laser_frame = last_laser_scan_->header.frame_id;
        rclcpp::Time stamp = last_laser_scan_->header.stamp;

        try {
            // Transform align point
            geometry_msgs::msg::PointStamped align_pt_laser;
            align_pt_laser.header.frame_id = laser_frame;
            align_pt_laser.header.stamp = stamp;
            align_pt_laser.point.x = align_x_laser;
            align_pt_laser.point.y = align_y_laser;
            align_pt_laser.point.z = 0.0;
            auto align_pt_global = tf_buffer_->transform(align_pt_laser, global_frame_, tf2::durationFromSec(1.0));

            // Transform entry point
            geometry_msgs::msg::PointStamped entry_pt_laser;
            entry_pt_laser.header.frame_id = laser_frame;
            entry_pt_laser.header.stamp = stamp;
            entry_pt_laser.point.x = entry_x_laser;
            entry_pt_laser.point.y = entry_y_laser;
            entry_pt_laser.point.z = 0.0;
            auto entry_pt_global = tf_buffer_->transform(entry_pt_laser, global_frame_, tf2::durationFromSec(1.0));

            // Transform center point
            geometry_msgs::msg::PointStamped center_pt_laser;
            center_pt_laser.header.frame_id = laser_frame;
            center_pt_laser.header.stamp = stamp;
            center_pt_laser.point.x = center_x_laser;
            center_pt_laser.point.y = center_y_laser;
            center_pt_laser.point.z = 0.0;
            auto center_pt_global = tf_buffer_->transform(center_pt_laser, global_frame_, tf2::durationFromSec(1.0));

            // Create static transforms for all three frames
            std::vector<geometry_msgs::msg::TransformStamped> transforms;

            // 1. cart_align_frame
            geometry_msgs::msg::TransformStamped align_tf;
            align_tf.header.stamp = this->now();
            align_tf.header.frame_id = global_frame_;
            align_tf.child_frame_id = "cart_align_frame";
            align_tf.transform.translation.x = align_pt_global.point.x;
            align_tf.transform.translation.y = align_pt_global.point.y;
            align_tf.transform.translation.z = 0.0;
            align_tf.transform.rotation.x = orientation.x();
            align_tf.transform.rotation.y = orientation.y();
            align_tf.transform.rotation.z = orientation.z();
            align_tf.transform.rotation.w = orientation.w();
            transforms.push_back(align_tf);

            // 2. cart_entry_frame (cart_frame for backwards compatibility)
            geometry_msgs::msg::TransformStamped entry_tf;
            entry_tf.header.stamp = this->now();
            entry_tf.header.frame_id = global_frame_;
            entry_tf.child_frame_id = "cart_frame";  // Keep original name
            entry_tf.transform.translation.x = entry_pt_global.point.x;
            entry_tf.transform.translation.y = entry_pt_global.point.y;
            entry_tf.transform.translation.z = 0.0;
            entry_tf.transform.rotation.x = orientation.x();
            entry_tf.transform.rotation.y = orientation.y();
            entry_tf.transform.rotation.z = orientation.z();
            entry_tf.transform.rotation.w = orientation.w();
            transforms.push_back(entry_tf);

            // 3. cart_center_frame
            geometry_msgs::msg::TransformStamped center_tf;
            center_tf.header.stamp = this->now();
            center_tf.header.frame_id = global_frame_;
            center_tf.child_frame_id = "cart_center_frame";
            center_tf.transform.translation.x = center_pt_global.point.x;
            center_tf.transform.translation.y = center_pt_global.point.y;
            center_tf.transform.translation.z = 0.0;
            center_tf.transform.rotation.x = orientation.x();
            center_tf.transform.rotation.y = orientation.y();
            center_tf.transform.rotation.z = orientation.z();
            center_tf.transform.rotation.w = orientation.w();
            transforms.push_back(center_tf);

            // Broadcast all transforms
            static_tf_broadcaster_->sendTransform(transforms);

            RCLCPP_INFO(this->get_logger(), "Published 3 cart frames:");
            RCLCPP_INFO(this->get_logger(), "  cart_align_frame  @ (%.2f, %.2f) in %s",
                        align_pt_global.point.x, align_pt_global.point.y, global_frame_.c_str());
            RCLCPP_INFO(this->get_logger(), "  cart_frame        @ (%.2f, %.2f) in %s",
                        entry_pt_global.point.x, entry_pt_global.point.y, global_frame_.c_str());
            RCLCPP_INFO(this->get_logger(), "  cart_center_frame @ (%.2f, %.2f) in %s",
                        center_pt_global.point.x, center_pt_global.point.y, global_frame_.c_str());

        } catch (tf2::TransformException& ex) {
            RCLCPP_ERROR(this->get_logger(), "TF2 transform failed: %s", ex.what());
            return;
        }
    }

    void control_loop() {
        if (!service_active_) {
            return;
        }

        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                              "Control loop active, state=%d", static_cast<int>(state_));

        auto twist_msg = geometry_msgs::msg::Twist();

        switch (state_) {
            case State::MOVING_TO_CART: {
                // Check if cart_frame is available
                if (!tf_buffer_->canTransform("robot_base_link", "cart_frame", tf2::TimePointZero)) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                         "Waiting for cart_frame TF to become available");
                    break;
                }

                // Use TF to get transform from robot_base_link to cart_frame
                geometry_msgs::msg::TransformStamped transform_stamped;
                try {
                    transform_stamped = tf_buffer_->lookupTransform(
                        "robot_base_link", "cart_frame", tf2::TimePointZero);
                } catch (tf2::TransformException& ex) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                         "TF lookup failed: %s", ex.what());
                    break;
                }

                // Get target position in robot frame
                double target_x = transform_stamped.transform.translation.x;
                double target_y = transform_stamped.transform.translation.y;

                // Calculate distance and angle to target
                double distance = sqrt(target_x * target_x + target_y * target_y);
                double angle_to_target = atan2(target_y, target_x);

                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                      "MOVING_TO_CART: distance=%.2f, angle=%.2f", distance, angle_to_target);

                // If very close, be more lenient with angle and just proceed
                if (distance < 0.10) {  // 10cm - close enough to move under shelf
                    vel_pub_->publish(twist_msg);  // Stop
                    state_ = State::MOVING_UNDER_SHELF;
                    RCLCPP_INFO(this->get_logger(), "Reached cart position (%.2fm away), moving under shelf", distance);
                    break;
                }

                // If far from target, prioritize alignment first
                if (distance > 0.3 && std::abs(angle_to_target) > 0.1) {  // 30cm away and >6 degrees off
                    twist_msg.angular.z = angle_to_target * 1.5;
                    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                          "Aligning: angular.z=%.2f", twist_msg.angular.z);
                } else {
                    // Close enough - move forward with gentle correction
                    twist_msg.linear.x = std::min(0.3, distance * 0.5);
                    twist_msg.angular.z = angle_to_target * 0.5;  // Gentle angular correction while moving
                    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                          "Approaching: linear.x=%.2f, angular.z=%.2f",
                                          twist_msg.linear.x, twist_msg.angular.z);
                }

                vel_pub_->publish(twist_msg);

                break;
            }

            case State::MOVING_UNDER_SHELF: {
                // Move forward 30cm
                if (!final_approach_started_) {
                    final_approach_start_x_ = current_x_;
                    final_approach_start_y_ = current_y_;
                    final_approach_started_ = true;
                }

                double distance_traveled = sqrt(
                    pow(current_x_ - final_approach_start_x_, 2) +
                    pow(current_y_ - final_approach_start_y_, 2));

                if (distance_traveled < 0.30) {  // 30cm
                    twist_msg.linear.x = 0.2;
                    vel_pub_->publish(twist_msg);
                    break;
                }

                vel_pub_->publish(twist_msg);  // Stop
                state_ = State::LIFTING_SHELF;
                RCLCPP_INFO(this->get_logger(), "Under shelf, lifting");

                break;
            }

            case State::LIFTING_SHELF: {
                // Lift the shelf
                auto msg = std_msgs::msg::String();
                elevator_up_pub_->publish(msg);

                RCLCPP_INFO(this->get_logger(), "Shelf lifted successfully");
                state_ = State::COMPLETED;
                approach_successful_ = true;
                service_active_ = false;
                break;
            }

            case State::COMPLETED: {
                // Do nothing
                break;
            }
        }
    }

    enum class State {
        MOVING_TO_CART,
        MOVING_UNDER_SHELF,
        LIFTING_SHELF,
        COMPLETED
    };

    // ROS2 components
    rclcpp::CallbackGroup::SharedPtr service_callback_group_;
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
    rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr service_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_up_pub_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;

    // State
    State state_ = State::COMPLETED;
    bool service_active_ = false;
    bool attach_to_shelf_ = false;
    bool approach_successful_ = false;
    bool final_approach_started_ = false;

    // Data
    sensor_msgs::msg::LaserScan::SharedPtr last_laser_scan_;
    double current_yaw_ = 0.0;
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double final_approach_start_x_ = 0.0;
    double final_approach_start_y_ = 0.0;

    // Parameters
    std::string global_frame_ = "map";
    std::string odom_frame_ = "odom";
    std::string base_frame_ = "robot_base_link";
    double intensity_thres_fact_ = 0.0;
    double intensity_threshold_ = 8000.0;
    double align_ahead_dist_ = 0.30;
    double align_cart_center_dist_ = 0.48;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ApproachServiceServer>();

    // Use MultiThreadedExecutor to allow service and timer to run concurrently
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
