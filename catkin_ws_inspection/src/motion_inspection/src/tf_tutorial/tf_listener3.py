#!/usr/bin/python

import rospy
import tf
from geometry_msgs.msg import PoseStamped

msg = None


def poseCallback(m):
    global msg
    m.header.stamp = rospy.Time(0)
    msg = m


if __name__ == '__main__':
    rospy.init_node("tf_listener")

    tf_listener = tf.TransformListener()
    pose_subscriber = rospy.Subscriber(
        "robot_pose", PoseStamped, callback=poseCallback)

    r = rospy.Rate(1)
    while not rospy.is_shutdown():

        if tf_listener.canTransform(target_frame="utm", source_frame="home", time=rospy.Time(0)) and msg is not None:

            res = tf_listener.transformPose(ps=msg, target_frame="utm")

            rospy.loginfo(res)

        else:
            rospy.logwarn("Cannot lookup transform between world and home")

        r.sleep()