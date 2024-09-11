#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <pointmatcher/PointMatcher.h>
#include <Eigen/Dense>
#include <cmath>
#include <vector>

using PM = PointMatcher<float>;
using DP = PM::DataPoints;

class ICPNode : public rclcpp::Node
{
public:
    ICPNode() : Node("icp_node")
    {
        // Subscribe to the /scan topic
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ICPNode::scanCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "ICP Node initialized and subscribing to /scan");
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received scan data");

        // Convert LaserScan to libpointmatcher DataPoints (2D point cloud)
        DP pointCloud = laserScanToPointCloud(scan_msg);

        // // For now, we can align the point cloud to itself as a demo
        // DP referenceCloud = pointCloud;

        // // Set up the ICP algorithm
        // PM::ICP icp;
        // icp.setDefault();  // Use default settings

        // // Perform ICP
        // PM::TransformationParameters transformation = icp(pointCloud, referenceCloud);

        // // Print the transformation matrix
        // std::stringstream ss;
        // ss << transformation.matrix().format(Eigen::IOFormat());
        // RCLCPP_INFO(this->get_logger(), "Transformation matrix:\n%s", ss.str().c_str());
    }

    DP laserScanToPointCloud(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        std::vector<float> ranges = scan_msg->ranges;
        float angle_min = scan_msg->angle_min;
        float angle_increment = scan_msg->angle_increment;

        // Prepare the point cloud
        DP::Labels labels;
        labels.push_back(PM::Label("x", 1));
        labels.push_back(PM::Label("y", 1));
        DP pointCloud(labels, ranges.size());

        // Convert ranges and angles to Cartesian coordinates
        for (size_t i = 0; i < ranges.size(); ++i)
        {
            if (std::isfinite(ranges[i]))  // Check for valid range values
            {
                float angle = angle_min + i * angle_increment;
                float x = ranges[i] * std::cos(angle);  // X = range * cos(angle)
                float y = ranges[i] * std::sin(angle);  // Y = range * sin(angle)
                pointCloud.features(0, i) = x;
                pointCloud.features(1, i) = y;
                pointCloud.features(2, i) = 1.0;  // Homogeneous coordinate
            }
            else
            {
                // If the range is not valid, set the point to zero
                pointCloud.features(0, i) = 0.0;
                pointCloud.features(1, i) = 0.0;
                pointCloud.features(2, i) = 1.0;
            }
        }

        return pointCloud;
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ICPNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
