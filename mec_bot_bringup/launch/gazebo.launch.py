import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    my_robot_dir = get_package_share_directory("mec_bot")

    # Declare argument for model path
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(my_robot_dir, "urdf", "mec_bot.urdf.xacro"),
        description="Absolute path to the robot URDF file"
    )

    # Declare argument for RViz config file
    rviz_config_arg = DeclareLaunchArgument(
        name="rviz_config",
        default_value=os.path.join(my_robot_dir, "rviz", "urdf_config.rviz"),
        description="Absolute path to RViz configuration file"
    )

    # Generate robot_description from Xacro
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str
    )

    # Start robot_state_publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen"
    )

    # Set Gazebo resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(my_robot_dir).parent.resolve())]
    )

    # Correctly form the path to the Gazebo launch file
    gazebo_launch_path = os.path.join(
        get_package_share_directory("ros_gz_sim"),
        "launch",
        "gz_sim.launch.py"
    )

    # Include Gazebo simulation launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments=[("gz_args", "-v 4 -r empty.sdf")]
    )

    # Spawn the robot in Gazebo
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "/robot_description", "-name", "mec_bot"]
    )

    # ROS-Gazebo Bridge for Communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'],
        output='screen'
    )

    # Launch RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")]
    )

    # Return launch description
    return LaunchDescription([
        model_arg,
        rviz_config_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo_resource_path,
        gazebo,
        bridge,
        gz_spawn_entity,
        rviz_node
    ])