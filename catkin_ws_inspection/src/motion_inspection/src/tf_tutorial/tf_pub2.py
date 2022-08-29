#!/usr/bin/python

import rospy
import tf
from geometry_msgs.msg import PoseStamped

class Robot(object):
   def __init__(self):
       position = PoseStamped()

       position.header.frame_id = "home"

       position.pose.position.x = -5
       position.pose.position.y = 10

       position.pose.orientation.w = 1

       self.position = position
       self.pub = rospy.Publisher("robot_pose", PoseStamped, queue_size=1)

   def publishPose(self):
       self.position.header.stamp = rospy.Time.now()

       self.pub.publish(self.position)

       

if __name__ == '__main__':
   rospy.init_node("tf_pub")

   robot = Robot()

   tf_broadcaster = tf.TransformBroadcaster()

   r = rospy.Rate(1)
   while not rospy.is_shutdown():

        tf_broadcaster.sendTransform(
            translation=[10, 10, 0],
            rotation=[0., 0., 0., 1],
            time=rospy.Time.now(),
            child="home",
            parent="world"
        )
        
        robot.publishPose()
        r.sleep()