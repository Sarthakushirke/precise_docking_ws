#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "pointmatcher/PointMatcher.h"
#include "tf2/LinearMath/Transform.h"
#include <iostream>
#include <sstream>  // For ostringstream
#include "frontend/matplotlibcpp.h"
#include "frontend/icp_mapping.h"
#include "tf2_ros/buffer.h"  // Include for TF buffer
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace plt = matplotlibcpp;

using namespace std;
typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

class ICPNode : public rclcpp::Node
{
public:
    ICPNode() : Node("icp_node"), map_builder() ,tf_buffer_(this->get_clock())
    {
        // Enable a dedicated thread for tf2 buffer
        tf_buffer_.setUsingDedicatedThread(true);
        // Create a subscription for point cloud data
        point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "transform_cloud", 10, std::bind(&ICPNode::pointCloudCallback, this, std::placeholders::_1));

        // Create a publisher for the transformed point cloud
        transformed_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("icp_transformed_cloud", 10);

        previous_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("icp_previous_cloud", 10);

        data_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("icp_data_cloud", 10);

        // Create the default ICP algorithm
        icp.setDefault();
    }

private:
    tf2_ros::Buffer tf_buffer_;  // Declare TF buffer
    // Callback when point clouds are received
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Received a new point cloud.");
        // Convert the ROS PointCloud2 message to libpointmatcher DataPoints
        DP data_cloud = rosMsgToDataPoints(msg);
        
        // If this is the first cloud, store it as the previous cloud and return
        if (!previous_cloud)
        {
            previous_cloud = data_cloud;
            RCLCPP_INFO(this->get_logger(), "I am out.");
            // printPointCloud(*previous_cloud, "previous_cloud");
            return;
        }

        // Compare the current cloud with the previous cloud
        if (data_cloud.features == previous_cloud->features)  // Check if the features are identical
        {
            RCLCPP_INFO(this->get_logger(), "Current cloud is identical to the previous cloud. Skipping ICP.");
            return;
        }

        // RCLCPP_INFO(this->get_logger(), "Received first cloud as previous.");

        // // Compute the transformation to express data in reference frame
        PM::TransformationParameters T = icp(data_cloud, *previous_cloud);

        // Convert matrix T to string for logging
        std::ostringstream oss;
        oss << T;

        // RCLCPP_INFO(this->get_logger(), "Published transformed cloud. Final transformation: \n%s", oss.str().c_str());


        // // Apply the transformation
        DP transformed_cloud(data_cloud);
        icp.transformations.apply(transformed_cloud, T);


        // Convert back to PointCloud2 and publish
        auto transformed_msg = dataPointsToRosMsg(transformed_cloud);
        transformed_cloud_publisher_->publish(transformed_msg);
        RCLCPP_INFO(this->get_logger(), "Published transformed cloud with %d points", transformed_cloud.features.cols());

        auto previous_msg = dataPointsToRosMsg(*previous_cloud);
        previous_cloud_publisher_->publish(previous_msg);
        RCLCPP_INFO(this->get_logger(), "Published previous cloud with %d points", previous_cloud->features.cols());

        auto data_msg = dataPointsToRosMsg(data_cloud);
        data_cloud_publisher_->publish(data_msg); 
        RCLCPP_INFO(this->get_logger(), "Published data cloud with %d points", data_cloud.features.cols()); 

        // Update previous_cloud to the current data cloud for the next callback
        previous_cloud = data_cloud;

        // plotMultiplePointClouds(*previous_cloud, data_cloud, transformed_cloud);

      

    }

    // Helper function to convert the transform from TF2 to Eigen matrix (for libpointmatcher)
    PM::TransformationParameters tfToEigen(const geometry_msgs::msg::TransformStamped &transform_stamped)
    {
        PM::TransformationParameters transform_matrix = PM::TransformationParameters::Identity(3, 3);

        transform_matrix(0, 2) = transform_stamped.transform.translation.x;
        transform_matrix(1, 2) = transform_stamped.transform.translation.y;

        tf2::Quaternion q(
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z,
            transform_stamped.transform.rotation.w);

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // Set rotation matrix for 2D transformation (yaw rotation only)
        transform_matrix(0, 0) = cos(yaw);
        transform_matrix(0, 1) = -sin(yaw);
        transform_matrix(1, 0) = sin(yaw);
        transform_matrix(1, 1) = cos(yaw);

        return transform_matrix;
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


    sensor_msgs::msg::PointCloud2 dataPointsToRosMsg(const DP& cloud)
    {
        // RCLCPP_INFO(this->get_logger(), "cloud.features.cols(): %d", cloud.features.cols());
        // RCLCPP_INFO(this->get_logger(), "cloud.features.rows(): %d", cloud.features.rows());

        sensor_msgs::msg::PointCloud2 cloud_msg;

        // Set the metadata
        cloud_msg.header.frame_id = "diff_drive/lidar_link";
        cloud_msg.height = 1; // Unorganized point cloud
        cloud_msg.width = cloud.features.cols(); // Number of points

        // Define point fields for x, y, z
        cloud_msg.fields.resize(3);
        cloud_msg.fields[0].name = "x";
        cloud_msg.fields[0].offset = 0;
        cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud_msg.fields[0].count = 1;

        cloud_msg.fields[1].name = "y";
        cloud_msg.fields[1].offset = 4;
        cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud_msg.fields[1].count = 1;

        cloud_msg.fields[2].name = "z";
        cloud_msg.fields[2].offset = 8;
        cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud_msg.fields[2].count = 1;

        // Set byte layout
        cloud_msg.is_bigendian = false;
        cloud_msg.point_step = 12;  // 12 bytes per point (3 floats: x, y, z)
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
        cloud_msg.is_dense = true;
        cloud_msg.data.resize(cloud_msg.row_step);

        // Fill in the data (copy x, y, z into the byte array)
        for (int i = 0; i < cloud.features.cols(); ++i)
        {
            float x = cloud.features(0, i);
            float y = cloud.features(1, i);
            float z = (cloud.featureExists("z")) ? cloud.features(2, i) : 0.0f;  // Default to 0 for 2D

            // Copy the floats into the `data` array in byte format
            memcpy(&cloud_msg.data[i * cloud_msg.point_step + 0], &x, sizeof(float)); // Copy x
            memcpy(&cloud_msg.data[i * cloud_msg.point_step + 4], &y, sizeof(float)); // Copy y
            memcpy(&cloud_msg.data[i * cloud_msg.point_step + 8], &z, sizeof(float)); // Copy z
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
        // RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
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
        // plt::scatter(trans_x, trans_y, 10.0, {{"color", "green"}, {"label", "Transformed Cloud"}});
        // Plot the transformed cloud with hollow circles
        plt::scatter(trans_x, trans_y, 10.0, {{"color", "green"}, {"marker", "o"}, {"facecolor", "none"}, {"label", "Transformed Cloud"}});


        // Set plot labels and grid
        plt::title("Point Clouds: Reference, Data, and Transformed");
        plt::xlabel("X");
        plt::ylabel("Y");
        plt::grid(true);
        plt::legend();

        // // Display the plot
        plt::pause(0.01);  // Pause to allow live updates
        plt::clf();  // Clear the plot for the next frame
        plt::show();
    }

    // Declare the MapBuilder object
    MapBuilder map_builder;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_cloud_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr data_cloud_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr previous_cloud_publisher_;

    PM::ICP icp;
    std::optional<DP> reference_cloud; // Store the reference point cloud
    std::optional<DP> previous_cloud; // Store the previous point cloud
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ICPNode>());
    rclcpp::shutdown();
    return 0;
}
