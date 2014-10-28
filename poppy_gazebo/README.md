Simple ROS package to run Poppy in Gazebo
=========

Tested under ROS Hydro/Ubuntu 12.04


- Go to the launch directory
- type `roslaunch poppy_flat_feet_gazebo.launch` this should run Gazebo with the Poppy model
- type `roslaunch poppy_flat_feet_control.launch` this should run the motor controllers

Then you have access to the controllers topics:
/poppy/[joint_name]_position_controller/command
/poppy/[joint_name]_position_controller/pid/parameter_descriptions
/poppy/[joint_name]_position_controller/pid/parameter_updates
/poppy/[joint_name]_position_controller/state

For example you can directly send a position command with:
`rostopic pub /poppy/r_shoulder_x_position_controller/command std_msgs/Float64 0.0`

The joints states (position, velocity and effort) are avaiable through the topic:
/poppy/joint_states

For example you can display these information with!
`rostopic echo /poppy/joint_states`

Finally a very simple python example to control the robot is provided in src/PoppySimpleROSMove.py
