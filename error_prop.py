#!/usr/bin/env python

import rospy
import math
import tf
import numpy as np
from geometry_msgs.msg import Pose
from jacobians import *
from turtlebot3_msgs.msg import SensorState
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from laser_feature_extraction.msg import DepthFeatures
from laser_feature_extraction.msg import LineMsg
from laser_feature_extraction.msg import CornerMsg

pose = Pose()
prev_right = 0
prev_left = 0
WHEEL_CIRCUMFERENCE = 0.2073 
TICKS_PER_REV = 4096.0
Rw = 0.1435 
lastData = None
xpos = 0
ypos = 0
dx = 0
dy = 0
delt_thed = 0
cov = np.mat('1 1 1; 1 1 1; 1 1 1', float)
corners = None
error = None
#ekf stuff---
u = [0,0]
wheel_encoder=[0,0]

class Line:

    # Coefficients are needed to find the distance from a point to the line
    # They are NOT needed to match one line to another line
    def __init__(self,A,B,C,p_a,p_b,id=-1):
        self.A = A
        self.B = B
        self.C = C
        self.p_a = p_a
        self.p_b = p_b
        self.id = id

        # If this is a valid line
        if p_a != -1 and p_b != -1:

            # Set line length and slope (m is basically A, but m is a better notation)
            # TH21F
            self.len = math.sqrt( pow(p_a.x - p_b.x,2) + pow(p_a.y - p_b.y,2) )
            self.m = (p_b.y - p_a.y) / (p_b.x - p_a.x)

            # Create a LineMsg object for publishing
            # TH21F
            self.msg = LineMsg(A,B,C,p_a,p_b,id)



    def printLine(self):
        #print('Line coefficients: [%s,%s,%s] length: %s Points on line: %s %s' % (self.A, self.B, self.C, self.len, self.p_a,self.p_b))
        zero = 0

    def __lt__(self, other):
        return self.p_a.x < other.p_a.x
    

class Corner:

    def __init__(self, p, psi, l_a, l_b, id=-1):
        #print('Making corner at p: %s psi: %s' % (p, psi))
        self.l_a = l_a
        self.l_b = l_b

        self.id = id

        # For data association
        # TH21F
        # Position of corner
        self.p = p
        # Angle of corner (angle between two lines forming corner)
        self.psi = psi

        # Msg to publish
        self.msg = CornerMsg(p, psi, self.id, l_a, l_b)

    def __eq__(self, other):
        return self.p == other.p
        '''
        if self.l_a == other.l_a or self.l_a == other.l_b:
            if self.l_b == other.l_a or self.l_b == other.l_b:
                return True
        return False
        '''

# Global rviz publiser for markers
# TH21F
pub_rviz = rospy.Publisher('visualization_marker', Marker, queue_size=10)
pub_features = rospy.Publisher('depth_features', DepthFeatures, queue_size=10)


###############################################################
#th Define the map
###############################################################

###############################################################
# Corners

# Points
p_one = Point(0.47, 0.04, 0.0)
p_two = Point(-1.35, -1.20, 0.0)
p_three = Point(-1.28, 1.64, 0.0)
p_four = Point(0.21, -0.42, 0.0)
p_five = Point(-1.47, 0.95, 0.0)
p_six = Point(-1.33, 1.32, 0.0)

# Angles
psi_one = 1.56
psi_two = 1.10
psi_three = 1.17
psi_four = 1.00
psi_five = 1.85
psi_six = 1.30

# Objects
c_one = Corner(p_one, psi_one, 0, 0, 1)
c_two = Corner(p_two, psi_two, 0, 0, 2)
c_three = Corner(p_three, psi_three, 0, 0, 3)
c_four = Corner(p_four, psi_four, 0, 0, 4)
c_five = Corner(p_five, psi_five, 0, 0, 5)
c_six = Corner(p_six, psi_six, 0, 0, 6)

# MAP
mapCorners = [c_one, c_two, c_three, c_four, c_five, c_six]
###############################################################21fth


###############################################################
# Lines
# Lengths



# End-points
a_one = Point(-1.60, 0.92, 0.0)
a_two = Point(-1.38, 1.53, 0.0)
a_three = Point(-1.34, -1.20, 0.0)
a_four = Point(-0.33, 1.90, 0.0)
a_five = Point(-0.14, 1.32, 0.0)

a_six = Point(-0.02, -0.47, 0.0)
a_seven = Point(0.29, -1.03, 0.0)
a_eight = Point(0.32, -0.36, 0.0)
a_nine = Point(0.54, 0.08, 0.0)
a_ten = Point(0.94, 1.93, 0.0)
a_eleven = Point(2.63, 0.04, 0.0)
a_twelve = Point(-1.04, 1.80, 0.0)

b_one = Point(-1.36, -1.20, 0.0)
b_two = Point(-1.32, 1.00, 0.0)
b_three = Point(-0.10, -1.08, 0.0)
b_four = Point(-1.18, 1.75, 0.0)
b_five = Point(-0.22, 1.41, 0.0)

b_six = Point(0.10, -0.48, 0.0)
b_seven = Point(0.81, -0.96, 0.0)
b_eight = Point(0.40, 0.0, 0.0)
b_nine = Point(0.20, 0.35, 0.0)
b_ten = Point(-0.10, 1.93, 0.0)
b_eleven = Point(2.40, 0.30, 0.0)
b_twelve = Point(-1.21, 1.73, 0.0)


# Objects
# Line coefficients are NOT needed to match one line to another line
l_one = Line(-1, -1, -1, a_one, b_one, 1)
l_two = Line(-1, -1, -1, a_two, b_two, 2)
l_three = Line(-1, -1, -1, a_three, b_three, 3)
l_four = Line(-1, -1, -1, a_four, b_four, 4)
l_five = Line(-1, -1, -1, a_five, b_five, 5)

l_six = Line(-1, -1, -1, a_six, b_six, 6)
l_seven = Line(-1, -1, -1, a_seven, b_seven, 7)
l_eight = Line(-1, -1, -1, a_eight, b_eight, 8)
l_nine = Line(-1, -1, -1, a_nine, b_nine, 9)
l_ten = Line(-1, -1, -1, a_ten, b_ten, 10)
l_eleven = Line(-1, -1, -1, a_eleven, b_eleven, 11)
l_twelve = Line(-1, -1, -1, a_twelve, b_twelve, 12)

# MAP
mapLines = [l_one, l_two, l_three, l_four, l_five, l_six, l_seven, l_eight, l_nine, l_ten, l_eleven, l_twelve]

#print('Map lines')
for l in mapLines:
    l.printLine()
###############################################################


# Returns a geometry_msgs/Point given a radian measure and distance measure from the scanner
def getPointFromRadAndDist(rad, dist):
    p = (dist * math.cos(rad), dist * math.sin(rad))

    point = Point()
    point.x = p[0]
    point.y = p[1]

    return point


# Return a line object with the two endpoints specified by p_a and p_b
def getLineBetweenPoints(p_a, p_b):
    #print('In getLineBetweenPoints')
    #print('Points: (%s,%s) (%s,%s)' % (p_a.x, p_a.y, p_b.x, p_b.y))

    # Get slope and constant (use either point to compute b)
    m = (p_b.y - p_a.y) / (p_b.x - p_a.x)
    b = p_a.y - (m*p_a.x)

    # Line coefficients
    A = m
    #B = (-b - (m*p_a.x)) / p_a.y
    B = -1
    C = b

    # Solve for the denominator to normalize the coefficients
    s = math.sqrt((A*A) + (B*B))
    
    #print('m: %s b: %s A: %s B: %s s: %s' % (m, b, A, B, s))

    # Normalize the terms. C has to be included despite it not being included to calculate s.
    A = A / s
    B = B / s
    C = C / s
    
    #print('Normalized A: %s B: %s' % (A,B))

    # Create the line object
    result = Line(A,B,C,p_a,p_b)

   #print('Exiting getLineBetweenPoints')
    return result



def getDistanceToLine(p, line):
    return abs(((line.A*p.x) + (line.B*p.y) + line.C))


def getDistBetwPoints(p_a, p_b):
    return math.sqrt( math.pow(p_a.x - p_b.x,2) + math.pow(p_a.y - p_b.y,2) )



# max dist to line
# index of point with max dist to line
# max dist between points
def getDistInfo(line, points):
    
   #print('Num points: %s' % len(points))

    i_maxDistToLine = 0
    maxDistToLine = 0
    maxDistBetwPoints = 0
    for i_p,p in enumerate(points):

        # Get perpendicular distance from line
        distToLine = getDistanceToLine(p, line)

        # Get dist to prev point
        distToPrevPoint = 0.0
        if i_p > 0:
            distToPrevPoint = getDistBetwPoints(p, points[i_p-1])
        
        if distToPrevPoint > maxDistBetwPoints:
            maxDistBetwPoints = distToPrevPoint
        
        # Track point with largest dist to line
        if distToLine > maxDistToLine:
            i_maxDistToLine = i_p
            maxDistToLine = distToLine
    
    return maxDistToLine, i_maxDistToLine, maxDistBetwPoints
   


def getLine(points, first=False):
    '''
    Given a list of geometry_msgs/Point objects, return the line that fits over them.

    Parameters:
        points (list): list of Point objects
        first (bool): True if the first time this is called, false otherwise.
                      This is needed so that we can use the point furthest away from the first Point object in the list
                      when this function is called for the first time.

    Returns:
        Line or -1: the Line obect that fits over the points, 
                    or -1 indicating that a line could not be found to fit the points.
        float or int: the max distance found between the line and any point in the points parameter,
                    or the index of the point with maximum distance from the line
    '''
    #print('In getLine')
    
    # Get index of last point in the list
    l = len(points)-1

    # If it's the first set of points, then don't use first and last points to find the line
    # Instead, use the first point and the point with furthest distance from the first point
    if first:
        # Loop through and find the point furthest away from the first point
        max_d = 0.0
        i_max_d = 0
        for i_p,p in enumerate(points):
            d = getDistBetwPoints(p, points[0])
            #print('points[0]: %s\n p: %s\n d: %s' % (points[0],p,d))
            if d > max_d:
                max_d = d
                i_max_d = i_p
        # Set 
        l = i_max_d
    
    #*2***1***f****t*****h********
    # Continue writing code below
    #*****************************  
      
    # Get the line endpoints
    a = points[0]
    b = points[l]

    # Get the line
    line = getLineBetweenPoints(a, b)

    # Get the distance info for the line and the points in the list
    maxDistToLine, i_maxDistToL, maxDistBetwPoints = getDistInfo(line, points)
    #print('i_max_dist: %s max dist: %s' % (i_max_dist_l, max_dist_to_line))

    # Evaluate the line based on a threshold
    # If either distance value is greater than the threshold then the line is invalid
    # TH21F
    threshold = 0.2
    if maxDistToLine > threshold or maxDistBetwPoints > threshold:
        line = Line(0,0,0,-1,-1)
        
    #print('Exiting getLine')
    return line, i_maxDistToL
    



# Return a set of geometry_msgs/Point objects that represent the laser points
def convertScanToCartPoints(data, out):

    for i in range(0, len(data.ranges)):
        
        if data.ranges[i] > 0:

            # Get the angle. This is in [0,2PI] range.
            angle = (i * data.angle_increment) + data.angle_min

            p = getPointFromRadAndDist(angle, data.ranges[i])

            if (not (math.isnan(p.x) or  math.isnan(p.y))):
                out.append(p)



def getAllLines(points):
    '''
    Given a list of geometry_msgs/Point objects, return all the lines 
    that can be fit over them

    Parameters:
        points (list): list of Point objects

    Returns:
        list: list of Line objects that were fit over the points parameter
    '''

    #print('In getAllLines')
    #print('21FTHpoints:')
    #print('%s,\n%s\n...\n%s' % (points[0], points[1], points[-1]))

    # Initialize an empty list to hold all the lines
    result = []

    # Get the first line over all the points
    # Do this outside the loop because the endpoints are different the first time we do this
    l, i_max_dist = getLine(points, True)
    result.append(l)
    
    #print('First line:')
    #result[0].printLine()

    # Maintain a list of point sets to process (create and evaluate a line for all sets of points in toProcess)
    toProcess = []
    
    # If the first line wasn't good then split the data and add the sets to toProcess
    if result[0].p_a == -1:
        #print('Line not good, adding to toProcess')

        # Split data on the point with max distance from the line
        p_a = points[0:i_max_dist]
        p_b = points[i_max_dist:]

        # Add new point sets to toProcess
        toProcess.append(p_a)
        toProcess.append(p_b)

        # Remove the bad line
        result = result[:-1]
    
    # Send points to be displayed in rviz (optional, debugging) 
    # TH21F
    #highlightPointSet(p_a,1)
    #highlightPointSet(p_b,2)
    #i_marker = 3


    # while there are still point sets to find lines for
    while len(toProcess) > 0:

        # Get next set of points to find a line for
        points = toProcess[0]

        # Send points to be displayed in rviz (optional, debugging) 
        #highlightPointSet(points, i_marker)
        #i_marker += 1

        # Check that we have more than 2 points. If we only have endpoints then no line should be valid.
        if len(toProcess[0]) > 2:

            #**************************************************
            # Calling getLine
            #**************************************************
            l, i_max_dist = getLine(toProcess[0])
            result.append(l)
            
            #print('New line:')
            #result[-1].printLine()
            
            # If the line is invalid, then remove it and split the data
            if result[-1].p_a == -1:
                # Split data
                p_a = points[0:i_max_dist]
                p_b = points[i_max_dist:]

                # Add new sets to toProcess list
                toProcess.append(p_a)
                toProcess.append(p_b)

                # Remove the bad line
                result = result[:-1]
            
        # Remove the first list in toProcess (the one we just processed)
        toProcess = toProcess[1:]

    #print('Exiting getAllLines')
    return result


def getAngleBetweenLines(l_a, l_b):
   #print('In getAngleBetweenLines')
    
    nom = l_a.A - l_b.A;
    denom = 1.0 + (l_a.A*l_b.A)

    theta = math.atan2(nom, denom)

   #print('l_a.A: %s l_b.A: %s nom: %s denom: %s theta: %s' % (l_a.A, l_b.A, nom, denom, theta))

   #print('Exiting getAngleBetweenLines')

    return theta


def getCornersFromLines(lines):
   #print('In getCornersFromLines')

    result = []

    lines = sorted(lines)

    for i_l,l in enumerate(lines):
        for i_m,m in enumerate(lines):

            if i_l != i_m:
                theta = getAngleBetweenLines(l, m)

                if abs(theta) > 0.785 and abs(theta) < 2.35:
                    d_aa = getDistBetwPoints(l.p_a, m.p_a)
                    d_ab = getDistBetwPoints(l.p_a, m.p_b)
                    d_ba = getDistBetwPoints(l.p_b, m.p_a)
                    d_bb = getDistBetwPoints(l.p_b, m.p_b)

                    # Check all four cases because we'll have to set corner position based on 
                    # which endpoints of the lines form a corner

                    if d_aa < 0.3:
                        # Get average of end-points to use as the corner position
                        p = Point()
                        p.x = (l.p_a.x + m.p_a.x) / 2.0
                        p.y = (l.p_a.y + m.p_a.y) / 2.0
        
                        psi = abs(l.A - m.A)

                        c = Corner(p,psi,l,m)
                    
                        if c not in result:
                            result.append(c)

                    elif d_ab < 0.3:
                        # Get average of end-points to use as the corner position
                        p = Point()
                        p.x = (l.p_a.x + m.p_b.x) / 2.0
                        p.y = (l.p_a.y + m.p_b.y) / 2.0

                        psi = abs(l.A - m.A)

                        c = Corner(p,psi,l,m)
                    
                        if c not in result:
                            result.append(c)

                    elif d_ba < 0.3:
                        # Get average of end-points to use as the corner position
                        p = Point()
                        p.x = (l.p_b.x + m.p_a.x) / 2.0
                        p.y = (l.p_b.y + m.p_a.y) / 2.0
                    
                        psi = abs(l.A - m.A)

                        c = Corner(p,psi,l,m)
                        
                        if c not in result:
                            result.append(c)


                    elif d_bb < 0.3:
                        # Get average of end-points to use as the corner position
                        p = Point()
                        p.x = (l.p_b.x + m.p_b.x) / 2.0
                        p.y = (l.p_b.y + m.p_b.y) / 2.0

                        psi = abs(l.A - m.A)

                        c = Corner(p,psi,l,m)

                        if c not in result:
                            result.append(c)


        '''
        # Find a way to sort the lines so that they would connect
        # Skip first line
        if i_l > 0:
            theta = getAngleBetweenLines(lines[i_l-1], l)

            if abs(theta) > 0.785:

                d = getDistBetwPoints(l.p_a, lines[i_l-1].p_b)

                if d < 0.2:
                    result.append(l.p_a)
        '''
            

    #buildRvizCorners(result)

   #print('Exiting getCornersFromLines')
    return result




def buildRvizLineList(lines):
    '''
    Creates a LineList Marker object and publishes it to Rviz.
    This function assumes there is a global Publisher created with the following line:
    pub_rviz = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    
    Parameters:
        lines (list): list of Line objects to visualize

    Returns:
        None
    '''
    #print('In buildLines')

    line_list = Marker()
    line_list.header.frame_id = 'base_scan'
    line_list.header.stamp = rospy.Time(0)
    line_list.ns = ''

    line_list.id = 0
    line_list.type = 5
    line_list.action = 0
    
    line_list.scale.x = 0.02

    line_list.color.g = 1.0
    line_list.color.a = 1.0

    # Add the line endpoints to list of points
    for l in lines:
        line_list.points.append(l.p_a)
        line_list.points.append(l.p_b)

    
    pub_rviz.publish(line_list)
    
    
    '''
    # Create text for each corner
    for i,l in enumerate(lines):
        textMarker = Marker()
        textMarker.header.frame_id = 'base_scan'
        textMarker.header.stamp = rospy.Time(0)
        textMarker.ns = ''

        textMarker.id = 200+i
        textMarker.type = 9
        textMarker.action = 0
        
        textMarker.scale.z = 0.1

        textMarker.color.r = 1.0
        textMarker.color.g = 1.0
        textMarker.color.b = 1.0
        textMarker.color.a = 1.0
        textMarker.colors.append(textMarker.color)
        

        textMarker.pose.position = Point( (l.p_a.x + l.p_b.x)/2.0, (l.p_a.y + l.p_b.y)/2.0, 0.0)
        textMarker.pose.position.z += 0.2

        textMarker.text = str(l.id)

        pub_rviz.publish(textMarker)
    '''

    #print('Exiting buildLines')




# Make an rviz line between points a and b
def buildRvizLine(a, b):
   #print('In buildLines')

    line_list = Marker()
    line_list.header.frame_id = 'base_scan'
    line_list.header.stamp = rospy.Time(0)
    line_list.ns = ''

    line_list.id = 0
    line_list.type = 5
    line_list.action = 0
    
    line_list.scale.x = 0.02

    line_list.color.g = 1.0
    line_list.color.a = 1.0


    line_list.points.append(a)
    line_list.points.append(b)

    
    pub_rviz.publish(line_list)

    

   #print('Exiting buildLines')


# Create rviz points for the first and last points in a set
# This helps visualize what line is being formed
# Color is adjusted between blue and red based on the id parameter
def highlightPointSet(points,id):
    pointMarker = Marker()
    pointMarker.header.frame_id = 'base_scan'
    pointMarker.header.stamp = rospy.Time(0)
    pointMarker.ns = ''

    pointMarker.id = id
    pointMarker.type = 8
    pointMarker.action = 0
    
    pointMarker.scale.x = 0.02
    pointMarker.scale.y = 0.02
    pointMarker.scale.z = 0.02

    pointMarker.color.b = 1.0 - (0.2*id)
    pointMarker.color.r = 0.0 + (0.2*id)
    pointMarker.color.a = 1.0
    pointMarker.colors.append(pointMarker.color)

    
    pointMarker.points.append(points[0])
    pointMarker.points.append(points[-1])

    pub_rviz.publish(pointMarker)

   #print('Exiting buildLines')




def buildRvizCorners(corners):
    '''
    Creates a LineList Marker object and publishes it to Rviz.
    This function assumes there is a global Publisher created with the following line:
    pub_rviz = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    
    Parameters:
        lines (list): list of Corner objects to visualize

    Returns:
        None
    '''
    pointMarker = Marker()
    pointMarker.header.frame_id = 'base_scan'
    pointMarker.header.stamp = rospy.Time(0)
    pointMarker.ns = ''

    pointMarker.id = 10
    pointMarker.type = 8
    pointMarker.action = 0
    
    pointMarker.scale.x = 0.1
    pointMarker.scale.y = 0.1
    pointMarker.scale.z = 0.1

    pointMarker.color.b = 1.0
    pointMarker.color.a = 1.0
        
    for c in corners:
        pointMarker.points.append(c.p)
        pointMarker.colors.append(pointMarker.color)

    pub_rviz.publish(pointMarker)

    '''
    # Create text for each corner
    for i,c in enumerate(corners):
        textMarker = Marker()
        textMarker.header.frame_id = 'base_scan'
        textMarker.header.stamp = rospy.Time(0)
        textMarker.ns = ''

        textMarker.id = 100+i
        textMarker.type = 9
        textMarker.action = 0
        
        textMarker.scale.z = 0.1

        textMarker.color.r = 1.0
        textMarker.color.g = 1.0
        textMarker.color.b = 1.0
        textMarker.color.a = 1.0
        textMarker.colors.append(textMarker.color)

        textMarker.pose.position = c.p
        textMarker.pose.position.z += 0.2

        textMarker.text = str(c.id)

        pub_rviz.publish(textMarker)
    '''
    

    #print('Exiting buildRvizCorners')


def findDistanceBetweenAngles(a, b):    
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


    

# Converts a list of Line objects and a list of Corner objects
# to a DepthFeatures msg
def convertObjectsToMsg(lines, corners, msg):
   #print('In convertObjecsToMsgs')

    for l in lines:
        msg.lines.append(l.msg)

    for c in corners:
        msg.corners.append(c.msg)    
    return msg.corners

   #print('Exiting convertObjecsToMsgs')




# Callback for laser sensing data
def laserCb(data):
   #print('In laserCb')

   #print('data: %s' % data)

    global corners
    global cov
    global error
    
    # Convert the data to a list of cartesian points
    points = []
    convertScanToCartPoints(data, points)

    # Get the list of lines for the points
    lines = getAllLines(points)

    ##print line info
    for l in lines:
        l.printLine()

    # Create an rviz line list and publish it
    buildRvizLineList(lines)

    corners = getCornersFromLines(lines)

    buildRvizCorners(corners)


    # Publish msg
    df = DepthFeatures()
    pub_features.publish(df)
    corners = convertObjectsToMsg(lines, corners, df)
    q = numpy.mat('.05 0.0 0.0; 0.0 .05 0.0; 0.0 0.0 .05',float)
    pPoint = Point()
    pPoint.x = pose.position.x
    pPoint.y = pose.position.y
    for c in corners:
        h = getH(c,pose,getDistBetwPoints(c.p,pPoint))
        error = h*cov*np.transpose(h)+q
    print error

# End callback

class State:
 
    def __init__(self, x, y, theta, vx, vy, vtheta):
        self.x = x
        self.y = y
        self.theta = theta
        self.vx = vx
        self.vy = vy
        self.vtheta = vtheta

x_prime = State(0,0,0,0,0,0)

def callback(data):
    global prev_right
    global prev_left
    global lastData
    global cov
    global u
    curr_right = data.right_encoder
    curr_left = data.left_encoder
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
        x_prime.vx = dx/float(data.header.stamp.to_sec() - lastData.header.stamp.to_sec())
        x_prime.vy = dy/float(data.header.stamp.to_sec() - lastData.header.stamp.to_sec())
        x_prime.vtheda = delt_thed/float(data.header.stamp.to_sec() - lastData.header.stamp.to_sec())

    lastData = data

    pose.position.x = x_prime.x
    pose.position.y = x_prime.y
    pose.orientation = tf.transformations.quaternion_from_euler(0,0,x_prime.theta)
    g = getG(pose,u)
    v = getV(pose,u)
    M = numpy.mat('0.0 0.0; 0.0 0.0',float)

    M[0,0] = .05*u[0]
    M[1,1] = .05*u[1]
    cov = (g*cov*np.transpose(g)) + (v*M*np.transpose(v))
    #print(cov)
    
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

#allyson time
def predict(data):
    print('in predict')
    global pose
    global u
    global cov
    global wheel_encoder
    
    prev_pose = pose        
    G = getG(prev_pose,u)   #Get the transition model matrix for the previous state and latest control
    V = getV(prev_pose,u)   #Get the motion model matrix for the previous state and latest control
    M = getM(u, 0.5)        #Get motion error matrix for motion model
    
    callback(data)
    cov = G * cov * np.transpose(G) + V * M * np.transpose(V)

    #testing
    print('POSE:', pose)
    print('COV:', cov)

#def measure():
    
 
def ekf_callback(data):
    print('in ekf callback')
    global wheel_encoder
    wheel_encoder = [data.left_encoder, data.right_encoder]


def main():
    print('In main')

    pose.position.x = 0
    pose.position.y = 0
    pose.position.z = 0
    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z= 0
    pose.orientation.w = 0

    #sub_lasers = rospy.Subscriber('/scan', LaserScan, laserCb)
    #rospy.Subscriber('/sensor_state', SensorState, callback)
    print('running sensor state')
    rospy.Subscriber('/sensor_state', SensorState, predict)
    rospy.spin()
    print('Exiting normally')


if __name__ == '__main__':
    try:
        #rospy.init_node('error_prop', anonymous=True)
        rospy.init_node('sensor_state', anonymous=True)
        main()
    except rospy.ROSInterruptException:
        pass

