from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution,AndSubstitution,PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node



def generate_launch_description():
    # use_sim_time = true for using gazebo time 
    task_id=LaunchConfiguration('task_id', default='1')
    sync_mapping=LaunchConfiguration('sync_mapping',default='false')
    localize=LaunchConfiguration('localize',default='false')
    
    # task 1 launch
    start_task_one = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tortoisebot_gazebo'),
                'launch',
                'gazebo_launch.py'
            ])
        ]),condition=IfCondition(PythonExpression(["'", task_id, "' == '1'"])))
    
    # task 2 launch
    start_task_two = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tortoisebot_nodes'),
                'launch',
                'find_closest_distance_launch.py'
            ])
        ]),condition=IfCondition(PythonExpression(["'", task_id, "' == '2'"])))
    
    # task 3 launch
    start_task_three = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tortoisebot_nodes'),
                'launch',
                'moving_sphere_launch.py'
            ])
        ]),launch_arguments={}.items(),
        condition=IfCondition(PythonExpression(["'", task_id, "' == '3'"])))
    
    # task 4 launch
    start_task_four = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tortoisebot_mapping'),
                'launch',
                'slam_launch.py'
            ])
        ]),launch_arguments={'sync_mapping':sync_mapping,'localize':localize,}.items(),
        condition=IfCondition(PythonExpression(["'", task_id, "' == '3'"])))
    
    

    return LaunchDescription([
        start_task_one,
        start_task_two,
        start_task_three,
        start_task_four,
    ])