The tb_control package represent an Open Controller node to govern a Turtlebot3 in the empty_world Gazebo environment. 

To run the code, perform the following steps:
1. Download this directory and unzip it in your <ROS_workspace>/src/ directory.
2. Go to the root directory of your ROS workspace and run "colcon build".
3. After successful build, run "source install/setup.bash".
4. Launch the turtlebot3 in the empty Gazebo world: ros2 launch turtlebot3_gazebo empty_world.launch.py
5. In a new terminal window, start the Open Controller node using scenario 1: ros2 run  tb_control  tb_openLoop "Scenario_1"
6. When finished with Scenario 1, enter "ctrl + c" in the terminal window where Scenario 1 was executed.
7. Start the Open Controller node using scenario 2: ros2 run  tb_control  tb_openLoop "Scenario_2"
