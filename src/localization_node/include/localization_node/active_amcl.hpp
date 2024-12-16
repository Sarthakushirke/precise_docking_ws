#ifndef ACTIVE_AMCL__ACTIVE_AMCL_HPP_
#define ACTIVE_AMCL__ACTIVE_AMCL_HPP_

#include "rclcpp/rclcpp.hpp"

#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace active_amcl
{
/**
 * @class active_pf_node
 * @brief ROS 2 node for Active Particle Filter Localization
 */
class active_pf_node : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for active_pf_node
   * @param options Node options for ROS 2 node configuration
   */
  explicit active_pf_node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor for active_pf_node
   */
  ~active_pf_node();

protected:
  /**
   * @brief Initialize ROS parameters
   */
  void initParameters();

  // Parameters
  double alpha1_;
  double alpha2_;
  double alpha3_;
  double alpha4_;
  double alpha5_;
};

}  // namespace active_amcl

#endif  // ACTIVE_AMCL__ACTIVE_AMCL_HPP_