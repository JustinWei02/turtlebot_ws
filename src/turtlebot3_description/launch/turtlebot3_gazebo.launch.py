#!/usr/bin/env python3
import os
from os import pathsep
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    IncludeLaunchDescription,
    EmitEvent,
    LogInfo,
    RegisterEventHandler
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    AndSubstitution,
    NotSubstitution
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.events import matches_action
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
    # Declare launch arguments.
    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Flag to enable RViz'
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='mapping.rviz',
        description='RViz config file (located in TurtleBot3 description package rviz directory)'
    )
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically start slam_toolbox lifecycle transitions'
    )
    use_lifecycle_manager_arg = DeclareLaunchArgument(
        'use_lifecycle_manager', default_value='false',
        description='Enable lifecycle manager (if true, manual activation required)'
    )
    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            get_package_share_directory("turtlebot3_description"),
            'config', 'slam_online.yaml'
        ),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node'
    )
    sdf_arg = DeclareLaunchArgument(
        'sdf',
        default_value=os.path.join(
            get_package_share_directory("turtlebot3_description"), "models", "turtlebot3", "model.sdf"
        ),
        description="Path to the SDF file containing both the world and the robot"
    )

    # Get package directories.
    turtlebot3_description = get_package_share_directory("turtlebot3_description")
    turtlebot3_description_prefix = get_package_prefix("turtlebot3_description")

    # Set the world path for Gazebo.
    world_path = os.path.join(turtlebot3_description, "models")
    world_path += pathsep + os.path.join(turtlebot3_description_prefix, "share")
    gz_sim_resource_path = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", world_path)
    gz_sim_render_engine = SetEnvironmentVariable("GZ_SIM_RENDER_ENGINE", "ogre2")

    # Start Gazebo.
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': LaunchConfiguration("sdf")}.items()
    )

    # Bridge nodes.
    bridge_cmd_vel = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist"],
        output="screen"
    )
    bridge_odometry = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/model/my_robot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry"],
        output="screen"
    )
    bridge_scan = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan"],
        output="screen"
    )
    bridge_tf = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'],
        output="screen"
    )

    # Publish a static transform from "my_robot/chassis" to "my_robot/chassis/lidar".
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0.15",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", "my_robot/chassis",
            "--child-frame-id", "my_robot/chassis/lidar"
        ],
        
        output='screen'
    )

    # Launch the odom_to_tf node (from your my_robot_tf package).
    # It publishes a transform from "my_robot/odom" to "my_robot/chassis".
    odom_tf_node = Node(
        package='my_robot_tf',
        executable='odom_to_tf',
        name='odom_to_tf_broadcaster',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Launch slam_toolbox as a LifecycleNode.
    slam_toolbox = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace='',
        output="screen",
        parameters=[
            LaunchConfiguration('slam_params_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'use_lifecycle_manager': LaunchConfiguration('use_lifecycle_manager'),
                'odom_frame': 'my_robot/odom',
                'base_frame': 'my_robot/chassis',
                'scan_topic': '/scan'
            }

        ]
    )

    # Lifecycle transitions (using the provided example code).
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox),
            transition_id=Transition.TRANSITION_CONFIGURE
        ),
        condition=IfCondition(LaunchConfiguration('autostart'))
    )
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Slam_toolbox node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_toolbox),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        ),
        condition=IfCondition(LaunchConfiguration('autostart'))
    )

    # Launch RViz2.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([turtlebot3_description, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    ld = LaunchDescription()

    # Add launch argument actions.
    ld.add_action(autostart_arg)
    ld.add_action(use_lifecycle_manager_arg)
    ld.add_action(rviz_launch_arg)
    ld.add_action(rviz_config_arg)
    ld.add_action(sim_time_arg)
    ld.add_action(slam_params_file_arg)
    ld.add_action(sdf_arg)

    # Add environment and simulation actions.
    ld.add_action(gz_sim_resource_path)
    ld.add_action(gz_sim_render_engine)
    ld.add_action(start_gazebo)

    # Add topic bridge nodes.
    ld.add_action(bridge_cmd_vel)
    ld.add_action(bridge_odometry)
    ld.add_action(bridge_scan)
    ld.add_action(bridge_tf)

    # Add static transform and odom_to_tf nodes.
    ld.add_action(static_tf)
    ld.add_action(odom_tf_node)

    # Add slam_toolbox and its lifecycle transitions.
    ld.add_action(slam_toolbox)
    ld.add_action(configure_event)
    ld.add_action(activate_event)

    # Add RViz node.
    ld.add_action(rviz_node)

    return ld

if __name__ == '__main__':
    generate_launch_description()
