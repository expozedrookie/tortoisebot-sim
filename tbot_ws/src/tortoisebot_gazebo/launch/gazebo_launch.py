from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution,FileContent,TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node



def generate_launch_description():
    # use_sim_time = true for using gazebo time 
    use_sim_time=LaunchConfiguration('use_sim_time', default='true')
    # urdf_file = name of the urdf can be dynamically given when launching this file which is already in install folder
    urdf_file=LaunchConfiguration('urdf_file', default='tortoisebot_Task2.urdf')
    # urdf_path = path of the urdf can be dynamically given when launching this file
    urdf_path=LaunchConfiguration('urdf_path', default=PathJoinSubstitution([FindPackageShare('tortoisebot_description'),'models', urdf_file]))
    # world_file = dynamic world name to load which is already in install folder
    world_file=LaunchConfiguration('world_file', default='empty_world.sdf')
    # world_location = dynamic path to the world file as per task
    world_path=LaunchConfiguration('world_path', default=PathJoinSubstitution([FindPackageShare('tortoisebot_gazebo'), 'worlds', world_file]))
    # rviz_file = dynamic file to the rviz_configuration file
    rviz_file=LaunchConfiguration('rviz_file', default='rviz.rviz')
    # rviz_config = dynamic path to the rviz_configuration file
    rviz_config=LaunchConfiguration('rviz_file_path', default=PathJoinSubstitution([FindPackageShare('tortoisebot_description'),'config', rviz_file]))
    # bridge_param_file  = name of the bridge file already in install folder
    bridge_param_file=LaunchConfiguration('bridge_param_file', default='bridge_parameters.yaml')
    # bridge_param_path = path to the bridge file for bridging topics ROS <-> GZ
    bridge_param_path=LaunchConfiguration('bridge_param_path', default=PathJoinSubstitution([FindPackageShare('tortoisebot_gazebo'),'config',bridge_param_file]))
    
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
            'gz_args':['-r ',world_path],
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
            '-x','0','-y','0', # spawn coordinates 
            '-z', '2'
            ],
            output='screen'
            )
    # start the gazebo ros bridge node
    gazebo_ros_bridge=Node(
        package='ros_gz_bridge',
		executable='parameter_bridge',
        parameters=[{
            'config_file':bridge_param_path # brdige param path given as parameter instead of ROS arguments, since it wont work when implementing Launch Configuration
        }],output='screen'
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