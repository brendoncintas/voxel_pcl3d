#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <unordered_set>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PointCloudToLaserScan : public rclcpp::Node
{
public:
  PointCloudToLaserScan()
    : Node("voxel_node")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "points",
      rclcpp::QoS(10),
      std::bind(&PointCloudToLaserScan::pointCloudCallback, this, std::placeholders::_1)
    );
    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      "voxel_scan",
      rclcpp::QoS(10)
    );
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Convert PointCloud2 message to PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // Voxelization parameters
    double voxel_size = 0.1;  // units are in meters

    // Voxelization process
    std::unordered_set<std::pair<int, int>, PairHash> voxel_grid;
    for (const auto& point : cloud->points)
    {
      int voxel_x = static_cast<int>(point.x / voxel_size);
      int voxel_y = static_cast<int>(point.y / voxel_size);
      voxel_grid.insert(std::make_pair(voxel_x, voxel_y));
    }

    // Create LaserScan message
    sensor_msgs::msg::LaserScan::SharedPtr laser_scan = std::make_shared<sensor_msgs::msg::LaserScan>();
    laser_scan->header = msg->header;
    laser_scan->angle_min = -M_PI / 2;
    laser_scan->angle_max = M_PI / 2;
    laser_scan->angle_increment = M_PI / 180;
    laser_scan->range_min = 0.0;
    laser_scan->range_max = std::numeric_limits<float>::infinity();
    laser_scan->ranges.clear();

    double max_range = 10.0;
    for (double angle = laser_scan->angle_min; angle <= laser_scan->angle_max; angle += laser_scan->angle_increment)
    {
      for (double distance = 0; distance < max_range; distance += voxel_size)
      {
        double x = distance * std::cos(angle);
        double y = distance * std::sin(angle);
        int voxel_x = static_cast<int>(x / voxel_size);
        int voxel_y = static_cast<int>(y / voxel_size);
        if (voxel_grid.count(std::make_pair(voxel_x, voxel_y)))
        {
          laser_scan->ranges.push_back(distance);
          break;
        }
        else if (distance + voxel_size >= max_range)
        {
          laser_scan->ranges.push_back(laser_scan->range_max);
        }
      }
    }

    // Publish LaserScan message
    publisher_->publish(*laser_scan);
  }

  struct PairHash
  {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2>& pair) const
    {
      std::size_t seed = 0;
      seed ^= std::hash<T1>{}(pair.first) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      seed ^= std::hash<T2>{}(pair.second) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      return seed;
    }
  };

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudToLaserScan>());
  rclcpp::shutdown();
  return 0;
}
