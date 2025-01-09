```bash
ros2 launch wheeltec_rviz2 wheeltec_rviz.launch.py
ros2 launch hugetank_nav2 tank_nav2.launch.py
ros2 launch stupid_car uwb_following.launch.py
cd ..
colcon build
source install/setup.zsh
ros2 launch lslidar_driver lslidar_c32_launch.py
cd code/ros2/dogshit
source install/setup.zsh
ros2 launch pointcloud_to_laserscan-humble pointcloud_to_laserscan_launch.py
ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py
cd code/ros2/dogshit
source install/setup.zsh
ros2 launch wheeltec_rviz2 wheeltec_rviz.launch.py
ros2 launch hugetank_link hugetank_link.launch.py
ros2 launch hugetank_nav2 tank_nav2.launch.py
ros2 topic echo cmd_vel
colcon build
cd ..
source install/setup.zsh
ros2 launch hugetank_nav2 tank_nav2.launch.py
colcon build
ros2 launch hugetank_nav2 tank_nav2.launch.py
ls
pwd
```

