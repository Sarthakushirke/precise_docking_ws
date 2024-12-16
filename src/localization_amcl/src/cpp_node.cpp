
#include "rclcpp/rclcpp.hpp"
#include "localization_amcl/cpp_header.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MycustomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}