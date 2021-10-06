#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

odom = Odometry()
t = Twist()
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
rate = None

def callback(data):
        global odom
        odom = data

def driveDistance(dist):
        global odom
        global t
        global pub
        global rate
        old_x = odom.pose.pose.position.x
        old_y = odom.pose.pose.position.y
        curr_dist = 0.0
        while curr_dist < dist:
                pub.publish(t)
                curr_dist = math.sqrt(((odom.pose.pose.position.x-old_x)**2)+((odom.pose.pose.position.y-old_y)**2))
                rate.sleep()
        t.linear.x = 0.0

def __main__():
        global t
        global pub
        global rate
        rospy.init_node('drive_robot', anonymous=True)
        rate = rospy.Rate(50) # 30hz
        rospy.Subscriber("odom",Odometry, callback)
        t.linear.x = 0.2
        t.angular.z = 0.0
        rospy.sleep(1)
        driveDistance(1)
        while not rospy.is_shutdown():
                #rospy.loginfo(t)
                pub.publish(t)
                rate.sleep()


if __name__ == '__main__':
        try:
                __main__()
        except rospy.ROSInterruptException:
                pass
