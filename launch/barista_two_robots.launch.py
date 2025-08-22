import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue 
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix

def generate_launch_description():

    # Declare launch argument
    include_laser_arg = DeclareLaunchArgument(
        'include_laser',
        default_value='true',
        description='Include laser scanner'
    )

    include_laser = LaunchConfiguration('include_laser')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    ####### DATA INPUT ##########
    xacro_file = 'barista_robot_model.urdf.xacro'
    package_description = "barista_robot_description"
    install_dir = get_package_prefix(package_description)

    # This is to find the models inside the models folder in package
    gazebo_models_path = os.path.join(package_description, 'models')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + \
            ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = install_dir + \
            "/share" + ':' + gazebo_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + \
            ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    print("GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH=="+str(os.environ["GAZEBO_PLUGIN_PATH"]))

    ####### DATA INPUT END ##########
    print("Fetching XACRO ==>")
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "xacro", xacro_file)

    rick_robot = "rick"
    rick_colour = "Gazebo/Blue"
    morty_robot = "morty"
    morty_colour = "Gazebo/Red"

    # Rick's Robot State Publisher
    rick_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        namespace=rick_robot,
        emulate_tty=True,
        parameters=[{
                'frame_prefix': rick_robot+'/',
                'use_sim_time': True, 
                'robot_description': ParameterValue(Command(['xacro ', robot_desc_path,
                                                            ' robot_name:=', rick_robot,
                                                            ' colour:=', rick_colour,
                                                            ' include_laser:=', include_laser]), 
                                    value_type=str)
            }],
        output="screen"
    )

    # Morty's Robot State Publisher
    morty_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        namespace=morty_robot,
        emulate_tty=True,
        parameters=[{
                'frame_prefix': morty_robot+'/',
                'use_sim_time': True, 
                'robot_description': ParameterValue(Command(['xacro ', robot_desc_path,
                                                            ' robot_name:=', morty_robot,
                                                            ' colour:=', morty_colour,
                                                            ' include_laser:=', include_laser]), 
                                    value_type=str)
            }],
        output="screen"
    )

    print("Fetching XACRO ==>  Rick & Morties robot_state_publisher DONE")

    # Joint State Publisher for rick
    rick_joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=rick_robot,
        parameters=[{'use_sim_time': use_sim_time}],
        output="screen"
    )
    # Joint State Publisher for Morty
    morty_joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=morty_robot,
        parameters=[{'use_sim_time': use_sim_time}],
        output="screen"
    )

    print("Fetching XACRO ==>  joint_state_publisher DONE")

    # Launch Gazebo with empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'verbose': 'false'}.items()
    )

    print("Fetching XACRO ==>  gazebo empty world DONE")

    # Spawn robot in Gazebo
    spawn_rick = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/'+rick_robot+'/robot_description',
                  '-entity', 'rick',
                  '-x', '0.5',
                  '-y', '0.0', 
                  '-z', '0.1'],
        output='screen'
    )
    
    spawn_morty = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/'+morty_robot+'/robot_description',
                  '-entity', 'morty',
                  '-x', '-0.5',
                  '-y', '0.0', 
                  '-z', '0.1'],
        output='screen'
    )

    print("Fetching XACRO ==>  spawning in gazebo world DONE")

    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'rick_and_morty.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config_dir] if os.path.exists(rviz_config_dir) else []
    )

    # Static transform publishers to connect rick and morty to world
    rick_world_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', rick_robot + '/odom'],
        
        output='screen'
    )
    morty_world_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher', 
        arguments=['0', '0', '0', '0', '0', '0', 'map', morty_robot + '/odom'],
        output='screen'
    )

    print("Fetching XACRO ==>  rviz DONE")

    # create and return launch description object
    return LaunchDescription([
        include_laser_arg,
        gazebo,
        rick_robot_state_publisher_node,
        morty_robot_state_publisher_node,
        rick_joint_state_publisher_node,  
        morty_joint_state_publisher_node,  
        spawn_rick,
        spawn_morty,
        rick_world_tf,
        morty_world_tf,
        rviz_node
    ])