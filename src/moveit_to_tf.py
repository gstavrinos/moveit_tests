#!/usr/bin/env python
import tf
import sys
import rospy
import moveit_msgs.msg
import moveit_commander
import geometry_msgs.msg

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_to_tf',)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group = moveit_commander.MoveGroupCommander('manipulator')

    # Override default planner
    group.set_planner_id('RRTConnectkConfigDefault')

    target_tf = rospy.get_param('~target_tf', '/target')
    base_tf = rospy.get_param('~base_tf', '/base_link')
    test_mode = rospy.get_param('~test_mode', False)
    x = rospy.get_param('~x', 0.1)
    y = rospy.get_param('~y', 0.4)
    z = rospy.get_param('~z', 0.8)

    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()

    rate = rospy.Rate(10.0)

    goal_reached = False

    while not (goal_reached or rospy.is_shutdown()):
        if test_mode:
            br.sendTransform((x, y, z),
                tf.transformations.quaternion_from_euler(0, 0, 0),
                rospy.Time.now(),
                target_tf,
                base_tf)
        try:
            (pos, quat) = listener.lookupTransform(base_tf, target_tf, rospy.Time.now())

            group.clear_pose_targets()

            pose_target = geometry_msgs.msg.Pose()
            pose_target.orientation.w = 1.0
            pose_target.position.x = pos[0]
            pose_target.position.y = pos[1]
            pose_target.position.z = pos[2]
            group.set_pose_target(pose_target)

            plan1 = group.plan()

            result = group.execute(plan1)

            #rate.sleep()

            rospy.sleep(5)

            print result

            if result:
                # TODO stop the routine when we reach the target
                # For now just pass
                pass

            rospy.sleep(1)
        except: #(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue