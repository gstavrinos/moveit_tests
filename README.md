tutorial.py
---
For this executable I used the following procedure and everything worked like a charm:

###### roslaunch ur_gazebo ur5.launch
###### roslaunch ur5_moveit_config moveit_rviz.launch config:=true
###### roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true
###### rosrun moveit_tests tutorial.py

moveit_to_tf.py
---
This script moves the end effector to the desired tf. You can change the test_mode to True to check the functionality without having to publish a tf. (moveit_to_tf.yaml)

For this executable I used the following procedure and everything worked like a charm:

###### roslaunch ur_gazebo ur5.launch
###### roslaunch ur5_moveit_config moveit_rviz.launch config:=true
###### roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true
###### roslaunch moveit_tests moveit_to_tf.launch
