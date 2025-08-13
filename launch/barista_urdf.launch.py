import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
import random

# this is the function launch  system will look for
def generate_launch_description():

    ####### DATA INPUT ##########
    urdf_file = 'barista_robot_model.urdf'
    #xacro_file = "urdfbot.xacro"
    package_description = "barista_robot_description"

    ####### DATA INPUT END ##########
    print("Fetching URDF ==>")
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "urdf", urdf_file)

    # Robot description ----------------
    # with open(robot_desc_path, 'r') as infp:
    #     robot_desc = infp.read()

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': Command(['xacro ', robot_desc_path])}],
        output="screen"
    )

    # Launch Gazebo with empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('barista_robot_description'), 'launch', 'start_cafe_world.launch.py')]),
        launch_arguments={'verbose': 'false'}.items()
    )

    # Spawn ROBOT Set Gazebo
    # Position and orientation
    position = [0.0, 0.0, 0.2]      # [X, Y, Z]
    orientation = [0.0, 0.0, 0.0]   # [Roll, Pitch, Yaw]
    # Base Name or robot
    robot_base_name = "barista_robot"

    entity_name = robot_base_name+"-"+str(int(random.random()*100000))
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity',
                   entity_name,
                   '-x', str(position[0]), '-y', str(position[1]
                                                     ), '-z', str(position[2]),
                   '-R', str(orientation[0]), '-P', str(orientation[1]
                                                        ), '-Y', str(orientation[2]),
                   '-topic', '/robot_description'
                   ]
    )


    # RVIZ Configuration
    rviz_config_file = os.path.join(get_package_share_directory(package_description), 'rviz', 'barista_model.rviz')

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            # arguments=['-d', rviz_config_dir],    
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [])

    # create and return launch description object
    return LaunchDescription(
        [
            gazebo,   
            robot_state_publisher_node,
            spawn_robot,
            rviz_node
        ]
    )
    