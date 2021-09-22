#!/usr/bin/env python
# license removed for brevity
import rospy
import math
import tf.transformations
from turtlebot3_msgs.msg import SensorState 
from nav_msgs.msg import Odometry 

prev_right = 0
prev_left = 0
WHEEL_CIRCUMFERENCE = 0.2073 
TICKS_PER_REV = 4096.0
Rw = 0.1435 
x_prime = State(0,0,0,0,0,0)
lastData = None
xpos = 0
ypos = 0
dx = 0
dy = 0
delt_thed = 0

def findDistanceBetweenAngles(a, b):
    '''
    Get the smallest orientation difference in range [-pi,pi] between two angles 
    Parameters:
        a (double): an angle
        b (double): an angle
    
    Returns:
        double: smallest orientation difference in range [-pi,pi]
    '''
    result = 0
    difference = b - a
    
    if difference > math.pi:
      difference = math.fmod(difference, math.pi)
      result = difference - math.pi
 
    elif(difference < -math.pi):
      result = difference + (2*math.pi)
 
    else:
      result = difference
 
    return result
 
 
 
def displaceAngle(a1, a2):
    '''
    Displace an orientation by an angle and stay within [-pi,pi] range
    Parameters:
        a1 (double): an angle
        a2 (double): an angle
    
    Returns:
        double: The resulting angle in range [-pi,pi] after displacing a1 by a2
    '''
    a2 = a2 % (2.0*math.pi)
 
    if a2 > math.pi:
        a2 = (a2 % math.pi) - math.pi
 
    return findDistanceBetweenAngles(-a1,a2)

def callback(data):
    global prev_right
    global prev_left
    global lastData
    print ('stuff)
    curr_right = data.right_encoder
    curr_left = data_encoder
    right_chng = 0
    left_chng = 0
    if prev_right != 0 :
        right_chng = curr_right - prev_right
        right_chng = right_chng/4096
    if prev_left != 0 :
        left_chng = curr_left - prev_left
        left_chng = left_chng/4096
    prev_right = data.right_encoder
    prev_left = data.left_encoder
    u = [left_chng * 0.2073,right_chng * 0.2073]
    x_prime = transitionModel(data,u)
    if lastData != None:
        x_prime.vx = dx/(data.header.time - lastData.header.time)
        x_prime.vy = dy/(data.header.time - lastData.header.time)
        x_prime.vtheda = delt_thed/(data.header.time - lastData.header.time)

    lastData = data

def callback2(data):
    global xpos
    global ypos
    xpos = data.pose.pose.position.x
    ypos = data.pose.pose.position.y


def listener():
    rospy.init_node('odom_compute', anonymous=True)

    rospy.Subscriber('/sensor_state', SensorState, callback)
    rospy.Subscriber('/odom', Odometry, callback2)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def buildOdomMsg(state, odomMsg):
        odomMsg.pose.pose.position.x = state.x
        odomMsg.pose.pose.position.y = state.y
        

def talker():
        global x_prime
        pub = rospy.Publisher('my_odom', myMsg,queue_size = 10)
        rate = rospy.Rate(30) # 10hz

        pub_msg = buildOdomMsg(x_prime,Odometry)
        while not rospy.is_shutdown():
                rospy.loginfo(pub_msg)
                pub.publish(pub_msg)
                rate.sleep()


if __name__ == '__main__':
    try:
            listener()
            talker()
    except rospy.ROSInterruptException:
            pass

class State:
 
    def __init__(self, x, y, theta, vx, vy, vtheta):
        self.x = x
        self.y = y
        self.theta = theta
        self.vx = vx
        self.vy = vy
        self.vtheta = vtheta

def transitionModel(x, u):
    global x_prime
    global dx
    global dy
    global delt_thed
    delt_thed = (u[1] - u[0])/(Rw*2)
    if delt_thed > math.pi:
      delt_thed = math.fmod(delt_thed, math.pi)
 
    elif(delt_thed < -math.pi):
      delt_thed = delt_thed + (2*math.pi)
    d = (u[0]+u[1])/2.0
    thed = (u[1] + u[0])/2.0*Rw
    dx = d*math.cos(thed+delt_thed)
    dy = d*math.sin(thed+delt_thed)
    x_prime = State(dx+xpos,dy+ypos,thed,0,0,0)
    return x_prime
        
