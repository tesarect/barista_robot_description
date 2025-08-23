# Create: launch/barista_single_robot.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # Package Directories
    xacro_file = 'barista_robot_model.urdf.xacro'
    package_description = "barista_robot_description"

    # Launch configuration variables
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='rick')
    colour_arg = DeclareLaunchArgument('colour', default_value='Gazebo/Red')
    include_laser_arg = DeclareLaunchArgument('include_laser', default_value='true')

    robot_name = LaunchConfiguration('robot_name')
    colour = LaunchConfiguration('colour')
    include_laser = LaunchConfiguration('include_laser')

    # Path to robot description
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "xacro", xacro_file)

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        parameters=[{
            'frame_prefix': 'rick/',
            'use_sim_time': False,  # Important: False for testing without Gazebo
            'robot_description': ParameterValue(Command(['xacro ', robot_desc_path,
                                                        ' robot_name:=', robot_name,
                                                        ' colour:=', colour,
                                                        ' include_laser:=', include_laser]), 
                                value_type=str)
        }],
        output="screen"
    )

    # Joint State Publisher GUI
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output="screen"
    )

    # RViz
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'rick_and_morty.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': False}],
        arguments=['-d', rviz_config_dir] if os.path.exists(rviz_config_dir) else []
    )

    return LaunchDescription([
        robot_name_arg,
        colour_arg,
        include_laser_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])