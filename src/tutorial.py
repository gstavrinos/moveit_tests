#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

# In this file, I am experimenting with, extending and adapting
# the Python MoveIt tutorial to the UR5 gazebo simulation
# glhf

# Current state of the code:
# Plan 1: Setting an ultimate goal point (Make sure that it is reachable!)
# Plan 2: Directly editing the joing positions
# Plan 3: A set of dummy waypoints

if __name__ == '__main__':
    print "============ Starting tutorial setup"
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group = moveit_commander.MoveGroupCommander("manipulator")

    # Override default planner
    group.set_planner_id("RRTConnectkConfigDefault")

    display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory,
                                    queue_size=20)

    rospy.sleep(1)

    print "============ Reference frame: %s" % group.get_planning_frame()
    print "============ End effector: %s" % group.get_end_effector_link()
    print "============ Robot Groups:"
    print robot.get_group_names()
    print "============ Printing robot state"
    print robot.get_current_state()
    print "============"



    print "============ Generating plan 1"
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = 1.0
    pose_target.position.x = 0
    pose_target.position.y = 0
    pose_target.position.z = 0.5
    group.set_pose_target(pose_target)

    plan1 = group.plan()
    print "============ Waiting while RVIZ displays plan1..."
    rospy.sleep(2)

    '''
    # Since group.plan() triggers the plan visualization
    # the code below is not needed.
    print "============ Visualizing plan1"
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()

    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan1)
    display_trajectory_publisher.publish(display_trajectory);
    rospy.sleep(5)
    '''

    print "============ Executing plan1"
    group.execute(plan1)

    rospy.sleep(2)
    
    group.clear_pose_targets()

    ## Then, we will get the current set of joint values for the group
    group_variable_values = group.get_current_joint_values()
    print "============ Joint values: ", group_variable_values

    ## Now, let's modify one of the joints, plan to the new joint
    ## space goal and visualize the plan
    for i in range(len(group_variable_values)):
        group_variable_values[i] = -1
    group.set_joint_value_target(group_variable_values)

    plan2 = group.plan()
    print "============ Executing plan2"
    group.execute(plan2)


    waypoints = []

    # The starting pose must not be added to the waypoints
    # MoveIt fails to find a plan if the starting point is included
    starting_pose = group.get_current_pose().pose

    # first orient gripper and move forward (+x)
    wpose = geometry_msgs.msg.Pose()
    wpose.orientation.w = 1.0
    wpose.position.x = starting_pose.position.x + 0.1
    wpose.position.y = starting_pose.position.y
    wpose.position.z = starting_pose.position.z
    waypoints.append(copy.deepcopy(wpose))

    # second move down
    wpose.position.z -= 0.10
    waypoints.append(copy.deepcopy(wpose))

    # third move to the side
    wpose.position.y += 0.05
    waypoints.append(copy.deepcopy(wpose))

    (plan3, fraction) = group.compute_cartesian_path(
                             waypoints,   # waypoints to follow
                             0.01,        # eef_step
                             0.0)         # jump_threshold

    print "============ Waiting while RVIZ displays plan3..."
    rospy.sleep(2)

    group.execute(plan3)