#include "rclcpp/rclcpp.hpp"
#include "localization_node/active_amcl.hpp"

#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>


namespace active_amcl
{
active_pf_node::active_pf_node(const rclcpp::NodeOptions & options)
: Node("active_pf_node", options)
{
  RCLCPP_INFO(this->get_logger(), "Active Particle Filter Node Creating");

  // Initialize parameters
  initParameters();

  // Log parameter values
  RCLCPP_INFO(this->get_logger(), "Alpha1: %f", alpha1_);
  RCLCPP_INFO(this->get_logger(), "Alpha2: %f", alpha2_);
  RCLCPP_INFO(this->get_logger(), "Alpha3: %f", alpha3_);
  RCLCPP_INFO(this->get_logger(), "Alpha4: %f", alpha4_);
  RCLCPP_INFO(this->get_logger(), "Alpha5: %f", alpha5_);

  // Additional initialization (e.g., setting up publishers/subscribers) can be done here
}

active_pf_node::~active_pf_node()
{
  RCLCPP_INFO(this->get_logger(), "Active Particle Filter Node Destroyed");
}

void active_pf_node::initParameters()
{
  // Declare parameters with default values
  alpha1_ = this->declare_parameter<double>("alpha1", 0.2);
  alpha2_ = this->declare_parameter<double>("alpha2", 0.2);
  alpha3_ = this->declare_parameter<double>("alpha3", 0.2);
  alpha4_ = this->declare_parameter<double>("alpha4", 0.2);
  alpha5_ = this->declare_parameter<double>("alpha5", 0.2);

  // If you need to perform additional validation or processing, do it here
}

}  // namespace active_amcl
