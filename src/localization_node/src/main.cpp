#include <memory>

#include "localization_node/active_amcl.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<active_amcl::active_pf_node>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}