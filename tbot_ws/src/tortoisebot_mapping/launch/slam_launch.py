from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription #,EmitEvent ,LogInfo,TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution,AndSubstitution,NotSubstitution #,  FileContent
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
# from launch_ros.actions import Node
from launch.conditions import IfCondition #,  UnlessCondition




def generate_launch_description():
    DeclareLaunchArgument('localize',default_value='false', description='true for localization')
    DeclareLaunchArgument('sync_mapping',default_value='false', description='true for online sync mapping')
    DeclareLaunchArgument('urdf_file',description='name of the urdf file')
    DeclareLaunchArgument('urdf_path',description='path of the urdf file')
    DeclareLaunchArgument('world_file',description='name of the world file')
    DeclareLaunchArgument('world_path',description='path of the world file')
    DeclareLaunchArgument('rviz_file',description='name of the rviz config file')
    DeclareLaunchArgument('rviz_file_path',description='path of the rviz config file')


    # argument for only enabling localization
    localize=LaunchConfiguration('localize')
    # argument for enabling sync mapping in slam_toolbox
    sync_mapping=LaunchConfiguration('sync_mapping')
    # argument for choosing urdf file (name)
    urdf_file=LaunchConfiguration('urdf_file', default='tortoisebot_Task4.urdf')
    # argument for choosing urdf file (path)
    urdf_path=LaunchConfiguration('urdf_path', default=PathJoinSubstitution([FindPackageShare('tortoisebot_description'),'models', urdf_file]))
    # argument for slam parameter file for any process - async mapping, sync mapping or localization - following three :
    sync_params_file=LaunchConfiguration('slam_params_file', default=PathJoinSubstitution([FindPackageShare('tortoisebot_mapping').find('tortoisebot_mapping'),'config', 'sync_map_config.yaml']))
    localize_params_file=LaunchConfiguration('slam_params_file', default=PathJoinSubstitution([FindPackageShare('tortoisebot_mapping').find('tortoisebot_mapping'),'config', 'localize_config.yaml']))
    async_params_file=LaunchConfiguration('slam_params_file', default=PathJoinSubstitution([FindPackageShare('tortoisebot_mapping').find('tortoisebot_mapping'),'config', 'async_map_config.yaml']))
    # argument for bridge file (name)
    bridge_param_file=LaunchConfiguration('bridge_param_file', default='mapping_bridge.yaml')
    # argument for world_file name
    world_file=LaunchConfiguration('world_file', default='custom_world.world.sdf')
    # argument for world file path
    world_path=LaunchConfiguration('world_path', default=PathJoinSubstitution([FindPackageShare('tortoisebot_gazebo').find('tortoisebot_gazebo'),'worlds', world_file]))
    # argument for world_file name
    rviz_file=LaunchConfiguration('rviz_file', default='rviz_task4.rviz')
    # argument for rviz_config path
    rviz_config=LaunchConfiguration('rviz_file_path', default=PathJoinSubstitution([FindPackageShare('tortoisebot_description'),'config', rviz_file]))
    
    #Start Gazebo + Rviz Launch file
    gazebo_rviz_launch_file=IncludeLaunchDescription( 
        PathJoinSubstitution(
            [FindPackageShare('tortoisebot_gazebo'),'launch','gazebo_launch.py']
            ),
            launch_arguments={
                'urdf_path':urdf_path,
                'world_path':world_path,
                'rviz_file_path':rviz_config,
                'bridge_param_file':bridge_param_file
            }.items()
    )
    
    # RUN IF LOCALIZE IS TRUE    
    slam_localization_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare('slam_toolbox'),'launch','localization_launch.py']
            ),
            launch_arguments={
                'slam_params_file':localize_params_file,
                'use_sim_time':'true'
            }.items(),
            condition=IfCondition(localize)
    )
    
    
    # RUN IF SYNC MAPPING AND LOCALIZE IS FALSE
    online_async_slam_mapping_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare('slam_toolbox'),'launch','online_async_launch.py']
            ),
            launch_arguments={
                'slam_params_file':async_params_file,
                'use_sim_time':'true'
            }.items(),
            condition=IfCondition(AndSubstitution(NotSubstitution(sync_mapping),NotSubstitution(localize)))
    )
    
    # RUN IF SYNC MAPPING IS TRUE AND LOCALIZE IS FALSE
    online_sync_slam_mapping_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare('slam_toolbox'),'launch','online_sync_launch.py']
            ),
            launch_arguments={
                'slam_params_file':sync_params_file,
                'use_sim_time':'true'
            }.items(),
            condition=IfCondition(AndSubstitution(sync_mapping,NotSubstitution(localize)))
    )

    
    return LaunchDescription([
        
        DeclareLaunchArgument('localize',default_value='false', description='true for localization'),
        DeclareLaunchArgument('sync_mapping',default_value='false', description='true for online sync mapping'),
        # DeclareLaunchArgument('urdf_file',description='name of the urdf file'),
        # DeclareLaunchArgument('urdf_path',description='path of the urdf file'),
        # DeclareLaunchArgument('world_file',description='name of the world file'),
        # DeclareLaunchArgument('world_path',description='path of the world file'),
        # DeclareLaunchArgument('rviz_file',description='name of the rviz config file'),
        # DeclareLaunchArgument('rviz_file_path',description='path of the rviz config file'),
        # DeclareLaunchArgument('slam_params_file',description='path of the slam_parameter file'),
        
        gazebo_rviz_launch_file,
        slam_localization_launch,
        online_sync_slam_mapping_launch,
        online_async_slam_mapping_launch,
        

    ])