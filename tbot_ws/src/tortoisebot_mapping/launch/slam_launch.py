from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,EmitEvent ,LogInfo,TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution,FileContent,AndSubstitution,NotSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node
from launch.conditions import UnlessCondition,IfCondition




def generate_launch_description():
    map_file_name=LaunchConfiguration('map_file_name', default='')
    localize=LaunchConfiguration('localize', default='false')
    sync_mapping=LaunchConfiguration('sync_mapping', default='false')

    urdf_file=LaunchConfiguration('urdf_file', default='tortoisebot_Task4.urdf')
    

    urdf_path=LaunchConfiguration('urdf_path', default=PathJoinSubstitution([FindPackageShare('tortoisebot_description'),'models', urdf_file]))
    
    sync_params_file=LaunchConfiguration('slam_params_file', default=PathJoinSubstitution([FindPackageShare('tortoisebot_mapping').find('tortoisebot_mapping'),'config', 'sync_map_config.yaml']))

    localize_params_file=LaunchConfiguration('slam_params_file', default=PathJoinSubstitution([FindPackageShare('tortoisebot_mapping').find('tortoisebot_mapping'),'config', 'localize_config.yaml']))

    async_params_file=LaunchConfiguration('slam_params_file', default=PathJoinSubstitution([FindPackageShare('tortoisebot_mapping').find('tortoisebot_mapping'),'config', 'async_map_config.yaml']))
    
    bridge_param_file=LaunchConfiguration('bridge_param_file', default='mapping_bridge.yaml')

    world_file=LaunchConfiguration('world_file', default='custom_world.world.sdf')
    world_path=LaunchConfiguration('world_path', default=PathJoinSubstitution([FindPackageShare('tortoisebot_gazebo').find('tortoisebot_gazebo'),'worlds', world_file]))
    
    rviz_config=LaunchConfiguration('rviz_file', default=PathJoinSubstitution([FindPackageShare('tortoisebot_description'),'config', 'rviz_Task4.rviz']))
    
    gazebo_rviz_launch_file=IncludeLaunchDescription( PathJoinSubstitution([FindPackageShare('tortoisebot_gazebo'),'launch','gazebo_launch.py']),launch_arguments={'urdf_path':urdf_path,'world_path':world_path,'rviz_file':rviz_config,'bridge_param_file':bridge_param_file}.items())
    
    # RUN IF LOCALIZE IS TRUE    
    slam_localization_launch = IncludeLaunchDescription( PathJoinSubstitution([FindPackageShare('slam_toolbox'),'launch','localization_launch.py']),launch_arguments={'slam_params_file':localize_params_file,'use_sim_time':'true'}.items(),condition=IfCondition(localize))
    
    
    # RUN IF SYNC MAPPING AND LOCALIZE IS FALSE
    online_async_slam_mapping_launch = IncludeLaunchDescription( PathJoinSubstitution([FindPackageShare('slam_toolbox'),'launch','online_async_launch.py']),launch_arguments={'slam_params_file':async_params_file,'use_sim_time':'true'}.items(),condition=IfCondition(AndSubstitution(NotSubstitution(sync_mapping),NotSubstitution(localize))))
    
    # RUN IF SYNC MAPPING IS TRUE AND LOCALIZE IS FALSE
    online_sync_slam_mapping_launch = IncludeLaunchDescription( PathJoinSubstitution([FindPackageShare('slam_toolbox'),'launch','online_sync_launch.py']),launch_arguments={'slam_params_file':sync_params_file,'use_sim_time':'true'}.items(),condition=IfCondition(AndSubstitution(sync_mapping,NotSubstitution(localize))))

    
    return LaunchDescription([
        
        gazebo_rviz_launch_file,
        slam_localization_launch,
        online_sync_slam_mapping_launch,
        online_async_slam_mapping_launch,
        

    ])