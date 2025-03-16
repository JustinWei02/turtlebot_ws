from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
    
    # Get TurtleBot3 description package directory
    turtlebot3_description = get_package_share_directory("turtlebot3_description")
    turtlebot3_description_prefix = get_package_prefix("turtlebot3_description")

    # Set the world path (for Gazebo resource finding)
    world_path = os.path.join(turtlebot3_description, "models")
    world_path += pathsep + os.path.join(turtlebot3_description_prefix, "share")

    # Set Gazebo resource path and force OGRE rendering
    gz_sim_resource_path = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", world_path)
    gz_sim_render_engine = SetEnvironmentVariable("GZ_SIM_RENDER_ENGINE", "ogre")

    # Declare the SDF file as a launch argument
    sdf_arg = DeclareLaunchArgument(
        name="sdf", 
        default_value=os.path.join(turtlebot3_description, "models", "turtlebot3", "model.sdf"),
        description="Path to the SDF file containing both the world and robot"
    )

    # Start Gazebo and load the SDF world file
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments=[
            ('gz_args', [LaunchConfiguration("sdf"),])
        ]
    )

    # Auto-bridge necessary topics
    bridge_cmd_vel = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist"],
        output="screen",
    )

    bridge_odometry = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/model/my_robot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry"],
        output="screen",
    )

    return LaunchDescription([
        gz_sim_resource_path,
        gz_sim_render_engine,  # Force OGRE rendering
        sdf_arg,
        start_gazebo,
        bridge_cmd_vel,  # Auto-bridge cmd_vel
        bridge_odometry  # Auto-bridge odometry
    ])
