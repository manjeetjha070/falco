#ifndef INCLUDED_FALCO_LOCAL_PLANNER_JOYSTICKINTERFACE_H
#define INCLUDED_FALCO_LOCAL_PLANNER_JOYSTICKINTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cmath>

class JoystickInterface : public rclcpp::Node
{
public:
    JoystickInterface ();

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy);


    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subJoy;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubJoyDir;

    bool twoWayDrive;
};

#endif // INCLUDED_FALCO_LOCAL_PLANNER_JOYSTICKINTERFACE_H


