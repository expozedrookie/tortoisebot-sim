from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration,FileContent,PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import UnlessCondition

def generate_launch_description():
	 # use_sim_time = true for using gazebo time when simulating .
	use_sim_time=LaunchConfiguration('use_sim_time',default='false')

	# urdf_path = path to the urdf for visualizing any URDF .
	urdf_path=LaunchConfiguration('urdf_path', default=PathJoinSubstitution([FindPackageShare('tortoisebot_description'),'models', 'tortoisebot.urdf']))
	#
	urdf_content=FileContent(urdf_path)
	# default rviz configuration file - for just showing robot model and TF
	rviz_config_location=PathJoinSubstitution([FindPackageShare('tortoisebot_description'),'config','rviz.rviz'])

	# RViz Node
	start_rviz=Node(
		package='rviz2',
		executable='rviz2',
		name='rviz2',
		arguments=['-d',rviz_config_location], # -d = Load Custom RViz Configuration File
		output='screen' # output prints in terminal else goes to log
	)
	# Robot State Publisher Node 
	start_rsp=Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		name='robot_state_publisher',
		parameters=[{'use_sim_time':use_sim_time,'robot_description':urdf_content}],
		output='screen'
	)
	# Joint State Publisher Node
	start_jsp=Node(
        package='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
		condition=UnlessCondition(use_sim_time), # Run only if use_sim_time is false
    )
	
	
	return LaunchDescription([
			start_rsp,
			start_jsp,
			start_rviz,
			])