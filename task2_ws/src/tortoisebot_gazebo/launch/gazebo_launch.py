from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution,FileContent
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os


def generate_launch_description():
    # use_sim_time = true for using gazebo time 
    use_sim_time=LaunchConfiguration('use_sim_time', default='true')
    # urdf_path = path of the urdf can be dynamically given when launching this file
    urdf_path=LaunchConfiguration('urdf_path', default=PathJoinSubstitution([FindPackageShare('tortoisebot_description'),'models', 'tortoisebot_Task2.urdf']))
    # world_location = dynamic path to the world file as per task
    world_location=LaunchConfiguration('world_path', default=PathJoinSubstitution([FindPackageShare('tortoisebot_gazebo'), 'worlds', 'empty_world.sdf']))
    # rviz_file = dynamic path to the rviz_configuration file
    rviz_config=LaunchConfiguration('rviz_file', default=PathJoinSubstitution([FindPackageShare('tortoisebot_description').find('tortoisebot_description'),'config', 'rviz.rviz']))
    # bridge_params = path to the bridge file for bridging topics ROS <-> GZ
    bridge_params=os.path.join(FindPackageShare('tortoisebot_gazebo').find('tortoisebot_gazebo'),'config','bridge_parameters.yaml')
    
    # Start Gazebo using ros_gz_sim launch
    start_gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args':['-r ',world_location],
            'on_exit_shutdown':'true'
            }.items()
            )
    # Spawn the robot model using create node in ros_gz_sim package
    create_entity =Node(
        package='ros_gz_sim',
        executable='create',
        name='spawnentity',
        arguments=[
            '-name', 'tortoisebot',
            #'-t','robot_description',
            '-file', urdf_path,
            '-z', '2'
            ],
            output='screen'
            )
    # start the gazebo ros bridge node
    gazebo_ros_bridge=Node(
        package='ros_gz_bridge',
		executable='parameter_bridge',
		arguments=[
            '--ros-args','-p', f'config_file:={bridge_params}' #bridge config file
            ],output='screen'
            )
    # start rviz launch file from
    rviz_launch_file=IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('tortoisebot_description'),'launch','rviz_launch.py']),
        launch_arguments={'use_sim_time':use_sim_time,'urdf_path':urdf_path,'config_file':rviz_config}.items()) # urdf path given as argument and the rviz config file

    return LaunchDescription([
        start_gazebo_world,
        create_entity,
        rviz_launch_file,
        gazebo_ros_bridge,
    ])