#include <rclcpp/rclcpp.hpp>

#include <falco_local_planner/JoystickInterface.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoystickInterface>());
    rclcpp::shutdown();
    return 0;
}
