from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo,TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution,FileContent
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node
from launch.conditions import UnlessCondition,IfCondition




def generate_launch_description():
    localize=LaunchConfiguration('localize', default='false')

    urdf_file=LaunchConfiguration('urdf_file', default='tortoisebot_Task4.urdf')
    
    

    urdf_path=LaunchConfiguration('urdf_path', default=PathJoinSubstitution([FindPackageShare('tortoisebot_description'),'models', urdf_file]))
    
    slam_params_file=LaunchConfiguration('slam_param_file', default=PathJoinSubstitution([FindPackageShare('tortoisebot_mapping').find('tortoisebot_mapping'),'config', 'mapping_config.yaml']))

    localize_params_file=LaunchConfiguration('slam_param_file', default=PathJoinSubstitution([FindPackageShare('tortoisebot_mapping').find('tortoisebot_mapping'),'config', 'localize_config.yaml']))
    
    bridge_param_file=LaunchConfiguration('bridge_param_file', default='mapping_bridge.yaml')

    world_file=LaunchConfiguration('world_file', default='custom_world.world.sdf')
    
    
    rviz_config=LaunchConfiguration('rviz_file', default=PathJoinSubstitution([FindPackageShare('tortoisebot_description'),'config', 'rviz_Task4.rviz']))
    
    gazebo_rviz_launch_file=IncludeLaunchDescription( PathJoinSubstitution([FindPackageShare('tortoisebot_gazebo'),'launch','gazebo_launch.py']),launch_arguments={'urdf_path':urdf_path,'world_file':world_file,'rviz_file':rviz_config,'bridge_param_file':bridge_param_file}.items())
    
    
    slam_localization_launch = IncludeLaunchDescription( PathJoinSubstitution([FindPackageShare('slam_toolbox'),'launch','localization_launch.py']),launch_arguments={'slam_params_file':localize_params_file,'use_sim_time':'true'}.items())

    slam_mapping_launch = IncludeLaunchDescription( PathJoinSubstitution([FindPackageShare('slam_toolbox'),'launch','online_async_launch.py']),launch_arguments={'slam_params_file':slam_params_file,'use_sim_time':'true'}.items())

    
    
    return LaunchDescription([
        
        gazebo_rviz_launch_file,
        # slam_mapping_launch,
        slam_localization_launch,
        

    ])