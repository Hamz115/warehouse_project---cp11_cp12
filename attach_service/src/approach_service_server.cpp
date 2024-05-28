#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "attach_service/srv/go_to_loading.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>
#include <cmath>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/detail/empty__struct.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include <future>

using GoToLoading = attach_service::srv::GoToLoading;
using LaserScan = sensor_msgs::msg::LaserScan;
using Twist = geometry_msgs::msg::Twist;
using String = std_msgs::msg::String;
using namespace std::placeholders;
using namespace std::chrono_literals;

class ApproachShelf : public rclcpp::Node {
public:
    // Methods
    ApproachShelf() : Node("approach_service_server") {
        // Create a subscription for LaserScan messages
        auto laser_callable = std::bind(&ApproachShelf::laser_callback, this, _1);
        _laser_subscription = this->create_subscription<LaserScan>("/scan", 1, laser_callable);
        // Create the service server
        auto callable_handle = std::bind(&ApproachShelf::handle_service, this, _1, _2);
        _service = this->create_service<GoToLoading>("approach_shelf", callable_handle);
        // Initialize transform buffer and listener
        _tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);
        // Intizalize static transform broadcaster
        _tf_static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        // Create a publisher for Twist anc Empty messages
        _publisher = this->create_publisher<Twist>("/cmd_vel", 1);
        _elevator_publisher = this->create_publisher<String>("/elevator_up", 1);
        // Inform server creation
        RCLCPP_INFO(this->get_logger(), "Service server started.");
    }
private:
    // General attributes
    rclcpp::Service<GoToLoading>::SharedPtr _service;
    rclcpp::Subscription<LaserScan>::SharedPtr _laser_subscription;
    rclcpp::Publisher<Twist>::SharedPtr _publisher;
    rclcpp::Publisher<String>::SharedPtr _elevator_publisher;

    // Scanning attributes
    int _num_legs = 0;
    int _total_readings;
    std::vector<std::pair<int, float>> _leg_data;
    float _scan_increment;

    // Transform attributes
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> _tf_static_broadcaster;

    // Final approach attributes

    // Methods
    void handle_service(const std::shared_ptr<GoToLoading::Request>, std::shared_ptr<GoToLoading::Response> response) {
        // Handle request
        RCLCPP_INFO(this->get_logger(), "Legs detected: %i.", _num_legs);
        RCLCPP_INFO(this->get_logger(), "L: %i R: %i.", _leg_data[0].first, _leg_data[1].first);
        
        if (_num_legs == 2) {
            // Publish cart frame
            RCLCPP_INFO(this->get_logger(), "Publishing cart frame.");
            this->publish_cart_frame();
            
            // Handle approach
            RCLCPP_INFO(this->get_logger(), "Approaching to cart.");
            if (!this->approach_cart("robot_front_laser_link", "cart_frame")) {
                RCLCPP_INFO(this->get_logger(), "The robot did not aligned to the cart correctly.");
                response->complete = false;
                return;
            }

            if (!this->approach_cart("robot_front_laser_link", "cart_center")) {
                RCLCPP_INFO(this->get_logger(), "The robot did not get under the cart correctly.");
                response->complete = false;
                return;
            }
            response->complete = true;
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "Not enough legs, aborting approach.");
            response->complete = false;
        }   
    }

    void laser_callback(const LaserScan::SharedPtr msg) {
        int legs = 0;
        _total_readings = int(msg->ranges.size());
        _leg_data.clear();
        bool leg_flag = false;
        int leg_start;
        for (int i = 0; i < int(msg->intensities.size()); i++) {
            if (msg->intensities[i] > 5000.0) {
                RCLCPP_DEBUG(this->get_logger(), "High reading at: %i.", i);
                if (!leg_flag) {
                    leg_flag = true;
                    leg_start = i;
                }
            }
            else {
                if (leg_flag) {
                    leg_flag = false;
                    int leg_center = (leg_start + i) / 2;
                    RCLCPP_DEBUG(this->get_logger(), "Leg center at: %i.", leg_center);
                    legs += 1;
                    _leg_data.push_back({leg_center, msg->ranges[leg_center]});
                }
            }
        }
        _num_legs = legs;
        _scan_increment = msg->angle_increment;
    }

    void publish_cart_frame() {
        // A: Robot to left leg
        float a = _leg_data[0].second;
        // B: Robot to right leg
        float b = _leg_data[1].second;
        // Theta: Angle between legs
        float theta = std::abs(_leg_data[0].first - _leg_data[1].first) * _scan_increment;
        // C: AB (Law of Cosines)
        float c = std::sqrt(a*a + b*b - 2*a*b * std::cos(theta));
        // D: Distance from robot to the middle of C (Apollonius Theorem)
        float d = 0.5 * std::sqrt(2*(a*a + b*b) - c*c);
        // Alpha: Angle for vertex A:C (Law of Sines)
        float alpha = std::asin(b * std::sin(theta) / c);
        // Beta: Angle for vertex A:D (Law of Sines)
        float beta = std::asin((c/2) * std::sin(alpha) / d);
        // Gamma: Angle from X+ to the middle of C
        float gamma = beta - std::abs(_leg_data[0].first - 540) * _scan_increment;
        // X: Coordinate x from laser link
        float x = d * std::cos(gamma);
        // Y: Coordinate y from laser link
        float y = d * std::sin(gamma);
        // Delta: Angle for vertex D:B
        float delta = theta - beta;
        // Lambda: Angle for vertex D:C/2 (Law of Sines)
        float lambda = std::asin(2 * b * std::sin(delta) / c);
        // Tau: Angle of correction for cart
        float tau = gamma + (M_PI/2 - lambda);

        RCLCPP_DEBUG(this->get_logger(), "X coordinate from laser: %.2f.", x);
        RCLCPP_DEBUG(this->get_logger(), "Y coordinate from laser: %.2f.", y);

        // Transform coordinates from laser to odom
        std::string laser_frame = "robot_front_laser_link";
        std::string parent_fame = "map";
        
        // Look up for the transformation between odom and laser frames
        int attempts = 0;
        bool success = false;
        do {
            attempts += 1;
            try {
                geometry_msgs::msg::TransformStamped odom_to_laser;
                odom_to_laser = _tf_buffer->lookupTransform(parent_fame, laser_frame, tf2::TimePointZero);

                geometry_msgs::msg::TransformStamped laser_to_cart;
                laser_to_cart.header.frame_id = laser_frame;
                laser_to_cart.child_frame_id = "cart_frame";
                laser_to_cart.transform.translation.x = x;  
                laser_to_cart.transform.translation.y = y;
                tf2::Quaternion q;
                q.setRPY(0, 0, tau);  // Set rotation in radians
                laser_to_cart.transform.rotation = tf2::toMsg(q);

                geometry_msgs::msg::TransformStamped cart_to_odom;
                tf2::doTransform(laser_to_cart, cart_to_odom, odom_to_laser);
                cart_to_odom.header.stamp = this->get_clock()->now();
                cart_to_odom.header.frame_id = parent_fame;
                cart_to_odom.child_frame_id = "cart_frame";

                _tf_static_broadcaster->sendTransform(cart_to_odom);

                geometry_msgs::msg::TransformStamped cart_to_center;
                cart_to_center.header.frame_id = "cart_frame";
                cart_to_center.child_frame_id = "cart_center";
                cart_to_center.transform.translation.x = 0.45; 

                geometry_msgs::msg::TransformStamped cart_center_to_odom;
                tf2::doTransform(cart_to_center, cart_center_to_odom, cart_to_odom);
                cart_center_to_odom.header.stamp = this->get_clock()->now();
                cart_center_to_odom.header.frame_id = parent_fame;
                cart_center_to_odom.child_frame_id = "cart_center";

                _tf_static_broadcaster->sendTransform(cart_center_to_odom);
                success = true;
            }
            catch (const tf2::LookupException & ex) {
                RCLCPP_INFO(this->get_logger(), "Could not perform some of the transformations. Please check the frame names.");
                RCLCPP_INFO(this->get_logger(), "%s", ex.what());
            }
            catch (const tf2::TransformException & ex) {
                RCLCPP_INFO(this->get_logger(), "Could not perform some of the transformations. Please check the frame names.");
                RCLCPP_INFO(this->get_logger(), "%s", ex.what());
            }
        } while (!success && attempts <= 10);
    }

    bool approach_cart(std::string origin_frame, std::string target_frame) {
        int attempts = 0;
        bool success = false;
        bool completed = false;
        do {
            attempts += 1;
            try {
                auto start_time = this->get_clock()->now();
                auto elapsed_time = this->get_clock()->now() - start_time;
                rclcpp::Duration timeout(25, 0);
                
                while (rclcpp::ok() && elapsed_time < timeout) {
                    elapsed_time = this->get_clock()->now() - start_time;
                    RCLCPP_DEBUG(this->get_logger(), "Elapsed time: %.2f seconds", elapsed_time.seconds());
                    geometry_msgs::msg::TransformStamped t;
                    t = _tf_buffer->lookupTransform(origin_frame, target_frame, tf2::TimePointZero);
                    // Compute velocities
                    auto x = t.transform.translation.x;
                    auto y = t.transform.translation.y;
            
                    float error_distance = std::sqrt(x*x + y*y);
                    float error_yaw = std::atan2(y, x);
                    RCLCPP_DEBUG(this->get_logger(), "Error distance: %.4f", error_distance);
                    RCLCPP_DEBUG(this->get_logger(), "Error yaw: %.4f", error_yaw);
                    Twist vel_msg;
                    if (error_distance > 0.02) {
                        //  Set velocity
                        vel_msg.angular.z = -1.0 * error_yaw;  
                        vel_msg.linear.x = std::min(2.0 * error_distance, 0.1);
                        RCLCPP_DEBUG(this->get_logger(), "Z angular: %.3f", vel_msg.angular.z);
                        RCLCPP_DEBUG(this->get_logger(), "X linear: %.3f", vel_msg.linear.x);
                        // Pubish velocity
                        _publisher->publish(vel_msg);
                        rclcpp::sleep_for(100ms);
                    }
                    else {
                        success = true;
                        vel_msg.angular.z = 0;
                        vel_msg.linear.x = 0;
                        _publisher->publish(vel_msg);
                        RCLCPP_INFO(this->get_logger(), "Approach to %s completed.", target_frame.c_str());
                        completed = true;
                        break;
                    }
                }      
            }
            catch (const tf2::LookupException & ex) {
                RCLCPP_INFO(this->get_logger(), "Could not perform some of the transformations. Please check the frame names.");
                RCLCPP_INFO(this->get_logger(), "%s", ex.what());
                rclcpp::sleep_for(1s);
            }
            catch (const tf2::TransformException & ex) {
                RCLCPP_INFO(this->get_logger(), "Could not perform some of the transformations. Please check the frame names.");
                RCLCPP_INFO(this->get_logger(), "%s", ex.what());
                rclcpp::sleep_for(1s);
            }
        } while (!success && attempts <= 10);
        return completed; 
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ApproachShelf>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}