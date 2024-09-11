#include "rclcpp/rclcpp.hpp"



class MycustomNode : public rclcpp::Node
{
    public:
    MycustomNode(): Node("my_node"){
        RCLCPP_INFO(this->get_logger(), "Test Cpp node");
    }

    private:
};