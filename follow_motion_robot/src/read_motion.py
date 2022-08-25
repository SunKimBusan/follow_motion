#!/usr/bin/python
import rospy
import numpy as np
from follow_motion_robot.msg import MotionData
import time
import rospkg

def motion_pub(data):
    pub = rospy.Publisher('motion_data', MotionData, queue_size=10)
    rospy.init_node('motion_data_node', anonymous=True)

    r = rospy.Rate(20)
    for i in range(len(data)):
        msg = MotionData()
        msg.Spine.x = data[i][7][0]
        msg.Spine.y = data[i][7][1]
        msg.Spine.z = data[i][7][2]
        msg.Nose.x = data[i][9][0]
        msg.Nose.y = data[i][9][1]
        msg.Nose.z = data[i][9][2]
        msg.Lshoulder.x = data[i][11][0]
        msg.Lshoulder.y = data[i][11][1]
        msg.Lshoulder.z = data[i][11][2]
        msg.Lelbow.x = data[i][12][0]
        msg.Lelbow.y = data[i][12][1]
        msg.Lelbow.z = data[i][12][2]
        msg.Lwrist.x = data[i][13][0]
        msg.Lwrist.y = data[i][13][1]
        msg.Lwrist.z = data[i][13][2]
        msg.Rshoulder.x = data[i][14][0]
        msg.Rshoulder.y = data[i][14][1]
        msg.Rshoulder.z = data[i][14][2]
        msg.Relbow.x = data[i][15][0]
        msg.Relbow.y = data[i][15][1]
        msg.Relbow.z = data[i][15][2]
        msg.Rwrist.x = data[i][16][0]
        msg.Rwrist.y = data[i][16][1]
        msg.Rwrist.z = data[i][16][2]
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    file_name = rospy.get_param('filename')
    rospack = rospkg.RosPack()
    motion_data = np.load(rospack.get_path('follow_motion_robot')+'/src/sample_motion/'+file_name+'.npy')
    motion_pub(motion_data)
