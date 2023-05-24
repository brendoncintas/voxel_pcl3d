import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Set the ROS2 namespace for the voxelization node
    voxelization_ns = launch.substitutions.LaunchConfiguration('voxelization_ns', default='voxelization')

    # Launch the voxelization node
    voxelization_node = Node(
        package='voxel_pcl3d',
        executable='pcl_voxel',
        name='voxel_node',
        output='screen',
        remappings=[
            ('points', '/velodyne_points'),
            ('scan', '/voxel_scan')
        ]
    )

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'voxelization_ns',
            default_value=voxelization_ns,
            description='Namespace for the voxelization node'
        ),
        voxelization_node,
    ])