#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <cmath>
#include <pcl/filters/statistical_outlier_removal.h>


class PointCloudVoxelizerNode : public rclcpp::Node
{
public:
  PointCloudVoxelizerNode() : Node("point_cloud_voxelizer")
  {
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/points", 10, std::bind(&PointCloudVoxelizerNode::pointcloudCallback, this, std::placeholders::_1));

    laserscan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/voxel_scan", 10);

    // Set the floor threshold
    floor_threshold_ = -0.2;  // Adjust this value based on your environment
  }

private:
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pcl_cloud);

    // Apply statistical outlier removal
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(pcl_cloud);
    sor.setMeanK(50);    // Number of neighbors to use for mean distance estimation
    sor.setStddevMulThresh(1.0);   // Standard deviation threshold
    sor.filter(*pcl_cloud);

    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(pcl_cloud);
    voxel_grid.setLeafSize(0.1, 0.1, 0.1); // Set the voxel grid size

    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_grid.filter(*downsampled_cloud);

    sensor_msgs::msg::LaserScan laserscan;
    laserscan.header = msg->header;

    // Define the laser scan parameters
    double angle_min = -M_PI / 4; // Minimum angle of the laser scan
    double angle_max = M_PI / 4;  // Maximum angle of the laser scan
    double angle_increment = (angle_max - angle_min) / downsampled_cloud->width; // Angle increment between each beam
    double range_min = 0.0;       // Minimum range
    double range_max = 100.0;     // Maximum range

    laserscan.angle_min = angle_min;
    laserscan.angle_max = angle_max;
    laserscan.angle_increment = angle_increment;
    laserscan.time_increment = 0.0; // Time increment between each beam (not used)
    laserscan.scan_time = 0.0;      // Total scan time (not used)
    laserscan.range_min = range_min;
    laserscan.range_max = range_max;

    size_t num_beams = std::round((angle_max - angle_min) / angle_increment) + 1;
    laserscan.ranges.resize(num_beams, range_max + 1.0); // Initialize ranges to be out of range

    // Convert 3D point cloud to laser scan
    for (const auto& point : downsampled_cloud->points)
    {
      if (point.z <= floor_threshold_)
      {
        // Skip points below the floor threshold
        continue;
      }

      double angle = std::atan2(point.y, point.x);
      double range = std::sqrt(std::pow(point.x, 2) + std::pow(point.y, 2) + std::pow(point.z, 2));

      // Check if the point falls within the laser scan field of view
      if (angle >= angle_min && angle <= angle_max)
      {
        // Calculate the corresponding beam index
        size_t beam_index = std::round((angle - angle_min) / angle_increment);

        // Update the range for the closest point in that beam
        if (range < laserscan.ranges[beam_index])
        {
          laserscan.ranges[beam_index] = range;
        }
      }
    }

    laserscan_pub_->publish(laserscan);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_pub_;

  double floor_threshold_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudVoxelizerNode>());
  rclcpp::shutdown();
  return 0;
}
