#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "falco_local_planner/msg/joystick_data.hpp"
#include <cmath>

#define PI 3.14159265

class JoystickInterface : public rclcpp::Node
{
public:
    JoystickInterface() : Node("joystick_interface")
    {
        // Parameters (same as in PathFollower for consistency)
        noRotAtStop       = this->declare_parameter<bool>("noRotAtStop", false);
        twoWayDrive       = this->declare_parameter<bool>("twoWayDrive", true);

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&JoystickInterface::joyCallback, this, std::placeholders::_1));

        joy_pub_ = this->create_publisher<falco_local_planner::msg::JoystickData>(
            "/joystick_data", 10);

        RCLCPP_INFO(this->get_logger(), "Joystick Interface Node started");
    }

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
    {
        auto msg = falco_local_planner::msg::JoystickData();
        msg.joy_time = this->now().seconds();

        // --- speed ---
        float joy_speed_raw = std::sqrt(joy->axes[3] * joy->axes[3] +
                                        joy->axes[4] * joy->axes[4]);
        msg.joy_speed = joy_speed_raw;
        if (msg.joy_speed > 1.0) msg.joy_speed = 1.0;
        if (joy->axes[4] == 0) msg.joy_speed = 0.0;

        // --- direction ---
        msg.joy_dir = 0.0;
        if (msg.joy_speed > 0.0) {
            msg.joy_dir = std::atan2(joy->axes[3], joy->axes[4]) * 180.0 / PI;
            if (joy->axes[4] < 0) msg.joy_dir *= -1;
        }

        if (joy->axes[4] < 0 && !twoWayDrive) {
            msg.joy_speed = 0;
            msg.joy_dir = 0.0;
        }

        // --- yaw (used in PathFollower) ---
        msg.joy_yaw = joy->axes[3];
        if (msg.joy_speed == 0.0 && noRotAtStop) {            
            msg.joy_yaw = 0.0;
        }

        if (joy->axes[2] > -0.1) {
            msg.autonomy_mode = false;
        } else {
            msg.autonomy_mode = true;
        }

        if (joy->axes[5] > -0.1) {
            msg.check_obstacle = true;
        } else {
            msg.check_obstacle = false;
        } 

        // publish
        joy_pub_->publish(msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<falco_local_planner::msg::JoystickData>::SharedPtr joy_pub_;
    bool noRotAtStop;
    bool twoWayDrive;
    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoystickInterface>());
    rclcpp::shutdown();
    return 0;
}
