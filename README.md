# trajopt_ros

This is a fork of the trajopt github with a WIP arm navigation package with the goal of allowing Moveit planning frameworks to use Trajopt to generate plans with forced orientation constraints on the end effector.

To make this work you will also need fetch_gazebo, fetchit from the uml repo,  fetch_ros,  gazebo, and tesseract in your catkin environment. 

For testing you can just:
rosrun arm_navigation arm_navigation node
after launching gazebo and moveit

#Issues
*The plans that are currently generated do not follow the desired trajectory, often leading to the robot flailing around randomly.
**Attempts to fix this that have failed include:
**Various configurations of different types of costs and constraints to force the planner to behave
**Different methods of acquiring the start joint configurations and poses, they work in their current configuration
**Using MoveIt to generate a cartesian path with orientation constraints first and/or exclusively
**Feeding the joint positions in different orders than the one given in gazebo/the urdf 
*Attempting to add time costs to the planner cause it to expect an additional joint, a dummy joint can be given to fix this

The issue likely is in the conversion from the trajopt plan into the moveit plan or the inverse kinematics for converting the goal pose into joint positions.