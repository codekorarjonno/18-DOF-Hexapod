# Laptop shortcuts for the hexapod (sourced from ~/.bashrc by install_laptop.sh)
# Real robot: ROS_DOMAIN_ID 42. Gazebo: 43 (the hexgz... shortcuts set it).

alias hexpi='ssh ariyan@hexapod.local'

# real robot
alias hexcon='source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 run hexapod_bringup console'
alias hexmapview='source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 launch hexapod_description map_view.launch.py'
alias hexnavview='rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz'
alias hexlocal='ros2 launch hexapod_bringup hexapod.launch.py'
alias hextwin='ros2 launch hexapod_description rviz.launch.py'
alias hexsim='source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 launch hexapod_bringup sim.launch.py'

# Gazebo simulation
alias hexgz='export ROS_DOMAIN_ID=43 && source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 launch hexapod_gazebo gazebo.launch.py'
alias hexgzmap='export ROS_DOMAIN_ID=43 && source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 launch hexapod_gazebo gazebo.launch.py slam:=true'
alias hexgznav='export ROS_DOMAIN_ID=43 && source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 launch hexapod_gazebo nav.launch.py slam:=true'
alias hexgznavmap='export ROS_DOMAIN_ID=43 && source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 launch hexapod_gazebo nav.launch.py'
alias hexgzbig='export ROS_DOMAIN_ID=43; ros2 launch hexapod_gazebo gazebo.launch.py slam:=true world:=big_world'
alias hexgzbignav='export ROS_DOMAIN_ID=43; ros2 launch hexapod_gazebo nav.launch.py slam:=true world:=big_world'
alias hexgzcon='export ROS_DOMAIN_ID=43 && hexcon'
hexgzsave() { ros2 run nav2_map_server map_saver_cli -f ~/maps/${1:-sim_map} --ros-args -p use_sim_time:=true; }   # hexgzsave [name]

# stop everything
alias hexkill='pkill -f ros2_ws/install; pkill -f /opt/ros/jazzy; pkill -f "gz sim"; ros2 daemon stop'
