#include <rclcpp/rclcpp.hpp>

#include <falco_local_planner/LocalPlanner.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalPlanner>();
    rclcpp::spin(node); 
    rclcpp::shutdown();
    return 0;
}
