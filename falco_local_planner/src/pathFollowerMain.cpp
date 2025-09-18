#include "pathFollower.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathFollower>();
  node->run();
  rclcpp::shutdown();
  return 0;
}
