#!/usr/bin/python

import rospy
import random
import math
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    rospy.init_node("tf_test")
    pub = rospy.Publisher("pose", PoseStamped, queue_size=1)

    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "frame1"

        msg.pose.position.x = random.randint(0, 5)
        msg.pose.position.y = random.randint(0, 5)
        msg.pose.position.z = 0

        quat = quaternion_from_euler(0., 0., math.radians(random.randint(0, 180)))

        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]

        pub.publish(msg)
        r.sleep()



