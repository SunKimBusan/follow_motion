#!/usr/bin/python

import rospy
import tf
from geometry_msgs.msg import PoseStamped


if __name__ == '__main__':
    rospy.init_node("tf_listener")

    tf_listener = tf.TransformListener()

    r = rospy.Rate(1)
    while not rospy.is_shutdown():

        if tf_listener.canTransform(
                target_frame="world", source_frame="home", time=rospy.Time(0)):
            # TF use Time(0) instead rospy.Time.now()

            tf_matrix = tf_listener.lookupTransform(
                target_frame="world", source_frame="home", time=rospy.Time(0))
            # return ([x, y, z], [x, y, z, w])

            rospy.loginfo(tf_matrix)

        else:
            rospy.logwarn("Cannot lookup transform between world and home")

        r.sleep()