Poppy MoveIt! Config
=====

Poppy's MoveIt integration 

How to install
====
* Add Poppy Control to your ROS workspace: https://forum.poppy-project.org/t/poppy-in-gazebot-and-v-rep/207/12
* Install MoveIt! through the package ros-hydro-moveit-full
* start the simulated robot: roslaunch poppy_gazebo poppy_flat_feet_gazebo.launch
* start the simulated controllers: roslaunch poppy_gazebo poppy_flat_feet_control.launch
* start the Moveit GUI and the planning pipeline: roslaunch poppy_moveit_config demo.launch
* In widget "Motion planning", tab "Context", Change the OMPL planning library from <unspecified> to RRTkConfigDefault
* Select a group in the widget "Displays", "Motion Planning", "Planning Request", "Planning group": eg "right_arm_with_torso" or keep the default one
* Use arrows and rings to modify a goal pose for the end effector of the selected group. Notice that orange links are the real-time IK and red links mean that they are in collision (your start state may collide already)
* click on "Plan". If planning succeeds the motion from start pose to goal pose will be simulated in loop
* "Plan and execute" should work also (slowly) if the model in Gazebo is in a valid state


Todo list
====
* Add a simpler collision model to the urdf

