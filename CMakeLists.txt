cmake_minimum_required(VERSION 3.5)
project(voxel_pcl3d)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)

# Add include directories
include_directories(include)

# Add executable
add_executable(pcl_voxel src/pcl_voxel.cpp)

# Link dependencies
ament_target_dependencies(pcl_voxel rclcpp sensor_msgs nav_msgs pcl_ros)

# Install executables
install(TARGETS pcl_voxel
        DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

# Install package.xml
install(FILES package.xml
        DESTINATION share/${PROJECT_NAME})

ament_package()