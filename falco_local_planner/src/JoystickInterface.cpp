#include <falco_local_planner/JoystickInterface.h>
#include <cmath>

JoystickInterface::JoystickInterface()
: Node("JoystickInterfacr")
{
    twoWayDrive             = declare_parameter<bool> ("twoWayDrive", true);

    subJoy = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&JoystickInterface::joyCallback, this, std::placeholders::_1));

    pubJoyDir = this->create_publisher<std_msgs::msg::Float32>("/joy_dir", 10);

}

void JoystickInterface::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
{    

    float joySpeedRaw = std::sqrt(joy->axes[3] * joy->axes[3] + joy->axes[4] * joy->axes[4]);
    float joySpeed = joySpeedRaw;

    if (joySpeed > 1.0f) joySpeed = 1.0f;
    if (joy->axes[4] == 0.0f) joySpeed = 0.0f;
    if (joy->axes[4] < 0.0f && !twoWayDrive) joySpeed = 0.0f;

    float joyDir = 0.0f;
    if (joySpeed > 0.0f) {
        joyDir = std::atan2(joy->axes[3], joy->axes[4]) * 180.0f / M_PI;
        if (joy->axes[4] < 0.0f) joyDir *= -1.0f;
    }

    std_msgs::msg::Float32 msg;
    msg.data = joyDir;
    pubJoyDir->publish(msg);

}
