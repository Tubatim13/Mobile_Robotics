#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from laser_feature_extraction.msg import DepthFeatures
from laser_feature_extraction.msg import LineMsg
from laser_feature_extraction.msg import CornerMsg
 
# Global rviz publiser for markers
pub_rviz = rospy.Publisher('visualization_marker', Marker, queue_size=10)
pub_features = rospy.Publisher('depth_features', DepthFeatures, queue_size=10)

class Line:
    A = None
    B = None
    C = None
    p_a = None
    p_b = None
    ID = 0
    msg = None

    def __init__(self,A,B,C,p_a,p_b,ID):
        self.A = A
        self.B = B
        self.C = C
        self.p_a = p_a
        self.p_b = p_b
        self.ID = ID
        self.msg = LineMsg(A,B,C,p_a,p_b,ID)

class Corner:
    p = Point()
    psi = 0.0
    ID = 0
    l_a = None
    l_b = None
    msg = None
        
    def __init__(self,p,psi,ID,l_a,l_b):
        self.p = p
        self.psi = psi
        self.ID = ID
        self.l_a = l_a
        self.l_b = l_b
        self.msg = CornerMsg(p,psi,ID,l_a,l_b)


def getLineBetweenPoints(points):
    p1 = points[0]
    p2 = points[1]
    a = 0
    if ((p2.x-p1.x) != 0):
        a = (p2.y-p1.y) / (p2.x-p1.x)
    b = -1
    c = p1.y-(a*p1.x)
    
    s = math.sqrt((a*a)+(b*b))

    a = a/s
    b = b/s
    c = c/s

    return Line(a,b,c,p1,p2,0)

def getDistanceToLine(point,line):
    return (line.A*point.x + line.B*point.x + line.C)
 

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
    print(len(lines))
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
    pointMarker.colors.append(pointMarker.color)

    for c in corners:
        pointMarker.points.append(c.p)
 
    pub_rviz.publish(pointMarker)
 
def getDistBetweenPoints(p1,p2):
    return math.sqrt(math.pow(p2.y-p1.y,2) + math.pow(p2.x-p1.x,2))
 
def getLine(points, first=False):
    '''
    Given a list of geometry_msgs/Point objects, return the line that fits over them.
     Parameters:
        points (list): list of Point objects
        first (bool): True if the first time this is called, false otherwise.
                      This is needed so that we can use the point furthest away from the first Point object in the list
                      when this function is called for the first time.
     Returns:
        Line or -1: the Line obect that fits over the points, or -1 indicating that a line could not be found to fit the points.
        float: the max distance found between the line and any point in the points parameter
    '''
    #print('In getLine')
    
    # Get index of last point in the list
    l = len(points)-1
    idx = 0
 
    # If it's the first set of points, then don't use first and last points to find the line
    # Instead, use the first point and the point with furthest distance from the first point
    if first:
        # Loop through and find the point furthest away from the first point
        max_d = 0.0
        i_max_d = 0
        for i_p,p in enumerate(points):
            d = getDistBetweenPoints(p, points[0])
            #print('points[0]: %s\n p: %s\n d: %s' % (points[0],p,d))
            if d > max_d:
                max_d = d
                i_max_d = i_p
                idx = points.index(p)
        # Set 
        l = i_max_d

    #*****************************
    # Continue writing code below
    #*****************************  
    line = getLineBetweenPoints([points[0],points[l]])
    maxdist = 0
    max_point = Point()
    for p in points:
        compare = line.A*p.x+line.B*p.y+line.C
        if(maxdist < compare):
                maxdist = compare
                max_point = p
    if (maxdist < .2 and getDistBetweenPoints(max_point,points[points.index(max_point)-1]) < .2):
        return line,maxdist
    else:
        return Line(0,0,0,-1,-1,0),points.index(max_point)
 
 
 
 
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
    #print('points:')
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
 
def getCornersFromLines(lines):
    corners = []
    lastLine = lines[0]
    for l in lines:
        slopeDiff = math.abs(lastLine.A-l.A)
        if(slopeDiff>45):
            endPointDist1 = math.sqrt(math.pow((lastLine.p_a.x-l.p_a.x),2)+math.pow((lastLine.p_a.y-l.p_a.y),2))
            endPointDist2 = math.sqrt(math.pow((lastLine.p_b.x-l.p_b.x),2)+math.pow((lastLine.p_b.y-l.p_b.y),2))
            #if(endPointDist1 > .3 or endPointDist2 > .3)
                    #corners.append(

def callback(data):
    points = []
    count = 0 
    for t in data.ranges:
        currPoint = Point()
        angle = data.angle_min + (count * data.angle_increment)
        currPoint.x = t * math.cos(angle)
        currPoint.y = t * math.sin(angle)
        points.append(currPoint)
        count += 1
    lines = getAllLines(points)
    buildRvizLineList(lines)
    #corners = getCornersFromLines(lines)
    #buildRvizCorners(corners)
    #pub_features.publish(DepthFeatures(lines,corners))

def main():
    rospy.init_node('depth_data', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    main()
