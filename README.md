tutorial.py
---
For this executable I used the following procedure and everything worked like a charm:

###### roslaunch ur_gazebo ur5.launch
###### roslaunch ur5_moveit_config moveit_rviz.launch config:=true
###### roslaunch ur5_moveit_config ur5_moveit_planning_execution
###### rosrun moveit_tests tutorial.py

Coming soon
---
Moving the end effector towards a tf
