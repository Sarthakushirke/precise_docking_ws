#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "pointmatcher/PointMatcher.h"
#include "tf2/LinearMath/Transform.h"
#include <iostream>
#include <sstream>  // For ostringstream
#include "frontend/matplotlibcpp.h"

namespace plt = matplotlibcpp;




using namespace std;
typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

class ICPNode : public rclcpp::Node
{
public:
    ICPNode() : Node("icp_node")
    {
        // Create a subscription for point cloud data
        point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "pointcloud", 10, std::bind(&ICPNode::pointCloudCallback, this, std::placeholders::_1));

        // Create a publisher for the transformed point cloud
        transformed_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("transformed_cloud", 10);

        // Create the default ICP algorithm
        icp.setDefault();
    }

private:
    // Callback when point clouds are received
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received a new point cloud.");
        // Convert the ROS PointCloud2 message to libpointmatcher DataPoints
        DP data_cloud = rosMsgToDataPoints(msg);

         // Print the data_cloud
        printPointCloud(data_cloud, "data_cloud");

        
        // // Assuming we already have the reference point cloud
        if (!reference_cloud)
        {
            reference_cloud = data_cloud;
            RCLCPP_INFO(this->get_logger(), "Received first cloud as reference.");
            printPointCloud(*reference_cloud, "reference_cloud");
            
            return;
        }

        // // Compute the transformation to express data in reference frame
        PM::TransformationParameters T = icp(data_cloud, *reference_cloud);

        // // Apply the transformation
        DP transformed_cloud(data_cloud);
        icp.transformations.apply(transformed_cloud, T);



        // Print the transformed cloud
        printPointCloud(transformed_cloud, "transformed_cloud");

        // Plot all clouds together
        plotMultiplePointClouds(*reference_cloud, data_cloud, transformed_cloud);


        // // Convert back to PointCloud2 and publish
        auto transformed_msg = dataPointsToRosMsg(transformed_cloud);
        transformed_cloud_publisher_->publish(transformed_msg);

        // RCLCPP_INFO(this->get_logger(), "Published transformed cloud. Final transformation: \n%s", T);
    }

    // Helper function to convert ROS PointCloud2 to libpointmatcher DataPoints
    DP rosMsgToDataPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Prepare feature labels: x, y (2D case)
        DP::Labels featureLabels;
        featureLabels.push_back(DP::Label("x", 1));
        featureLabels.push_back(DP::Label("y", 1));

        // Number of points
        size_t num_points = msg->width * msg->height;

        // Initialize a matrix for the features (x, y, and 1 for homogeneous coordinates in 2D)
        Eigen::MatrixXf features(3, num_points); // 3 rows for x, y, and homogeneous 1.0

        // Extract the point cloud data from the byte array
        for (size_t i = 0; i < num_points; ++i)
        {
            // Compute the byte offset for each point
            size_t point_offset = i * msg->point_step;

            // Extract x and y values from the byte array using struct unpack
            float x, y;
            memcpy(&x, &msg->data[point_offset + msg->fields[0].offset], sizeof(float));  // x field
            memcpy(&y, &msg->data[point_offset + msg->fields[1].offset], sizeof(float));  // y field

            // Store the points in the feature matrix (x, y, and homogeneous coordinate 1.0)
            features(0, i) = x;
            features(1, i) = y;
            features(2, i) = 1.0;  // Homogeneous coordinate for 2D
        }

        // Create the DataPoints object with the extracted features
        DP dataPoints(features, featureLabels);

        return dataPoints;
    }


    // Helper function to convert libpointmatcher DataPoints back to ROS PointCloud2
    sensor_msgs::msg::PointCloud2 dataPointsToRosMsg(const DP& cloud)
    {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        // Set the metadata
        cloud_msg.header.frame_id = "map";
        cloud_msg.height = 1;
        cloud_msg.width = cloud.features.cols();

        // Define point fields
        cloud_msg.fields.resize(3);
        cloud_msg.fields[0].name = "x";
        cloud_msg.fields[1].name = "y";
        cloud_msg.fields[2].name = "z";

        cloud_msg.is_bigendian = false;
        cloud_msg.point_step = 12;  // 3 floats (x, y, z)
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
        cloud_msg.is_dense = true;
        cloud_msg.data.resize(cloud_msg.row_step);

        // Fill in the data
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

        for (int i = 0; i < cloud.features.cols(); ++i, ++iter_x, ++iter_y, ++iter_z)
        {
            *iter_x = cloud.features(0, i);
            *iter_y = cloud.features(1, i);
            if (cloud.featureExists("z"))
                *iter_z = cloud.features(2, i);
            else
                *iter_z = 0.0;  // No z value in 2D clouds
        }

        return cloud_msg;
    }


    void printPointCloud(const DP& cloud, const std::string& cloud_name)
    {
        std::ostringstream oss;
        oss << "Point Cloud: " << cloud_name << "\n";
        oss << "Number of points: " << cloud.features.cols() << "\n";
        for (int i = 0; i < cloud.features.cols(); ++i)
        {
            oss << "Point " << i << ": x = " << cloud.features(0, i) << ", y = " << cloud.features(1, i);
            if (cloud.featureExists("z"))
            {
                oss << ", z = " << cloud.features(2, i);
            }
            oss << "\n";
        }
        RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
    }

    // Function to live plot point clouds
    void plotMultiplePointClouds(const DP& reference_cloud, const DP& data_cloud, const DP& transformed_cloud)
    {
        std::vector<float> ref_x, ref_y, data_x, data_y, trans_x, trans_y;

        // Extract points from reference cloud
        for (int i = 0; i < reference_cloud.features.cols(); ++i)
        {
            ref_x.push_back(reference_cloud.features(0, i));
            ref_y.push_back(reference_cloud.features(1, i));
        }

        // Extract points from data cloud
        for (int i = 0; i < data_cloud.features.cols(); ++i)
        {
            data_x.push_back(data_cloud.features(0, i));
            data_y.push_back(data_cloud.features(1, i));
        }

        // Extract points from transformed cloud
        for (int i = 0; i < transformed_cloud.features.cols(); ++i)
        {
            trans_x.push_back(transformed_cloud.features(0, i));
            trans_y.push_back(transformed_cloud.features(1, i));
        }

        // Plot reference cloud in red
        plt::scatter(ref_x, ref_y, 10.0, {{"color", "red"}, {"label", "Reference Cloud"}});
        
        // Plot data cloud in blue
        plt::scatter(data_x, data_y, 10.0, {{"color", "blue"}, {"label", "Data Cloud"}});
        
        // Plot transformed cloud in green
        plt::scatter(trans_x, trans_y, 10.0, {{"color", "green"}, {"label", "Transformed Cloud"}});

        // Set plot labels and grid
        plt::title("Point Clouds: Reference, Data, and Transformed");
        plt::xlabel("X");
        plt::ylabel("Y");
        plt::grid(true);
        plt::legend();

        // Display the plot
        plt::pause(0.01);  // Pause to allow live updates
        plt::clf();  // Clear the plot for the next frame
    }


    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_cloud_publisher_;

    PM::ICP icp;
    std::optional<DP> reference_cloud; // Store the reference point cloud
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ICPNode>());
    rclcpp::shutdown();
    return 0;
}
