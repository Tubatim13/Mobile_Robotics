#!/usr/bin/env python

import rospy
import math
import numpy as np
from geometry_msgs.msg import Pose

pose = Pose()

def callback(data):
    pose.position = data.pose.pose.position
    pose.orientation = data.pose.pose.orientation
    g = getG(pose.position,cov)
    v = getV(pose.position,cov)
    cov = g*cov*np.transpose(g) + v*.05*np.transpose(v)
    print(cov)
    

def main():
    pose.position.x = 0
    pose.position.y = 0
    pose.position.z = 0
    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z= 0
    pose.orientation.w = 0
    cov = np.mat('1 1 1; 1 1 1; 1 1 1', float)

    rospy.Subscriber('my_odom', Pose, callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        rospy.init_node('error_prop', anonymous=True)
        main()
    except rospy.ROSInterruptException:
        pass
