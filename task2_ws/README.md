# Task 2 


- Go to task<number>_ws directory -``cd task2_ws``
- Build the workspace             - ``colcon build``
- Source the workspace            - ``source install/setup.bash``

**Task 2 - Get nearest object distance using LiDAR and laser filters**
- Closest Distance using custom filter - ``ros2 launch tortoisebot_filters find_closest_distance_launch.py ``
- Closest Distance using laser_filters - ``ros2 launch tortoisebot_filters find_closest_distance_launch.py  use_laser_filters:=true ``

