# My_robot
A complete ROS 2 Humble package for simulating, visualizing, teleoperating, mapping (SLAM), and navigating a custom mobile robot in Gazebo.


✅ 1. Launch robot model in RViz
bash
Copier
Modifier
ros2 launch my_robot_description display.launch.py

This command launches RViz2 with the robot's URDF model using the display.launch.py file from the my_robot_description package.
It visualizes the robot structure, frames (TFs), and any sensors defined in the URDF, helping verify that the robot model is correctly defined and displayed.

✅ 2. Launch robot in Gazebo simulation environment

ros2 launch my_robot_description robot_gazebo.launch.py world:=./src/my_robot_description/world/world_test.world use_sim:=true

This command starts the robot in the Gazebo simulation environment, loading the custom world world_test.world.
The use_sim:=true flag ensures that ROS nodes use the simulation clock instead of the real-time system clock.
It’s the first step for testing SLAM or navigation in a virtual environment.

✅ 3. Control the robot manually with keyboard

ros2 run teleop_twist_keyboard teleop_twist_keyboard

This command runs a terminal-based teleoperation node, allowing you to manually control the robot using your keyboard (WASD keys).
It sends velocity commands (/cmd_vel) to the robot — perfect for exploration and mapping while using SLAM.

✅ 4. Run SLAM to build a map online

ros2 launch slam_toolbox online_async_launch.py \
params_file:=~/ros2_ws/src/my_robot_description/config/mapper_params_online_async.yaml \
use_sim_time:=true

This launches SLAM Toolbox in online asynchronous mode, using your custom configuration file.
The robot will build a map in real-time while it explores the environment.
The use_sim_time flag ensures synchronization with Gazebo’s simulation clock.

✅ 5. Launch Nav2 for autonomous navigation

ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true

This command starts the Nav2 (Navigation 2) stack, enabling autonomous navigation.
It includes global and local planners, costmaps, and the lifecycle manager.
Once a map is available, you can send navigation goals via RViz and the robot will plan and follow a path autonomously.
