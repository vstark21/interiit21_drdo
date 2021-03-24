#!/usr/bin/env python

import rospy
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandTOL, CommandBool, SetMode
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, PoseStamped, Twist, TwistStamped
from sensor_msgs.msg import Image
import math, cv2
import threading
import sys, select, termios, tty
import tf
from cv_bridge import CvBridge
import os
import cv2
import math
import numpy as np
from cv2 import aruco
import time
from interiit21_drdo.msg import Setpoints


pi_2 = math.pi / 2.0
global KILL_THREAD
KILL_THREAD = False

img=None
pos=None
r=None
p=None
y=None
class Controller:

    def __init__(self,aruco=None):
        rospy.init_node('camera_node')
        rospy.Subscriber("/mavros/state", State, self.state_callback)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pos_callback)
        # Better to comment these lines, unless you need them
        # rospy.Subscriber("/depth_camera/rgb/image_raw", Image, self.dpcamrgb_callback)
        # rospy.Subscriber("/depth_camera/depth/image_raw", Image, self.dpcam_callback)

        rospy.Subscriber("/camera/color/image_raw", Image, self.downcam_callback)
        self.cmd_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)

       
        self.aruco = aruco
        self.pose = Pose()
        self.state = State()
        self.timestamp = rospy.Time()
        self.bridge = CvBridge()
        self.down_cam=np.zeros((640,400,3),np.uint8)

    def state_callback(self, data):
        self.state = data
        # print(data)

    def pos_callback(self, data):
        self.timestamp = data.header.stamp
        print data.header.frame_id
        self.pose = data.pose
        global pos
              
        pos=[ data.pose.position.x, data.pose.position.y, data.pose.position.z]
        global r,p,y
        quats = [self.pose.orientation.w,
                self.pose.orientation.x,
                self.pose.orientation.y,
                self.pose.orientation.z]
        r,p,y = tf.transformations.euler_from_quaternion(quats)
       
    
    def dpcamrgb_callback(self, data):
        try:
            bridge = CvBridge()
            image1 = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')[:, :, ::-1]
            cv2.imshow("Forward_rgb", image1)
            cv2.waitKey(1)
        except Exception as e:
            rospy.loginfo(e)
    
    def dpcam_callback(self, data):
        try:
            bridge = CvBridge()
            image2 = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
            cv2.imshow("Forward_depth", image2)
            cv2.waitKey(1)
        except Exception as e:
            rospy.loginfo(e)

    def downcam_callback(self, data):
       
        bridge = CvBridge()
        image3 = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')[:, :, ::-1]
        
        
        
        self.down_cam=image3
        if self.aruco is not None:
            global y,pos
            pos = self.aruco.Main(y, [self.pose.position.x, self.pose.position.y, self.pose.position.z], self.down_cam)
            print pos
            if pos != None:
                x, y_w, z = pos
                self.goto_xyz_rpy(x, y_w, z, 0, 0, pi_2)

            #cv2.imshow("Downward_rgb", image3)
            #cv2.waitKey(1)

    def goto(self, pose):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id='map'
        pose_stamped.pose = pose
        

        self.cmd_pos_pub.publish(pose_stamped)

    def goto_xyz_rpy(self, x, y, z, ro, pi, ya):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        quats = tf.transformations.quaternion_from_euler(ro, pi, ya + pi_2)

        pose.orientation.x = quats[0]
        pose.orientation.y = quats[1]
        pose.orientation.z = quats[2]
        pose.orientation.w = quats[3]
        self.goto(pose)

class Aruco_Land():
    # Constructor
    def __init__(self):
        self.Limits_Initialized = False # To initialize the left right limits for zig-zag motion as per drone position
        self.Landing = False # Flag for whether we found the correct aruco, and it's time to land
        self.Landed = False # Flag for whether we finally landed after finding the correct aruco
        self.Aruco_Position = None # To store the correct aruco position
        self.Front_Limit = 2.6 # How much to traverse length in forward direction while in zig-zag motion
        self.Left_Right_Limit = 1.5 # How much to traverse length in left and right direction while in zig-zag motion
        self.Aruco_Length = 1.1 # Threshold length for aruco / white point detection, To avoid visiting the same aruco again
        self.Drone_Height = 3.0 # Drone height to maintain while searching the aruco
        self.Factor = 0.0046 # Factor to scale between image co-ordinate system and real world co-ordinate system
        self.Threshold_Distance = 0.3 # Threshold distance to check whether drone reached the desired position
        
        self.Visited_Aruco = [] # To keep track record of visited aruco
        self.Aruco_Dict = aruco.Dictionary_get(aruco.DICT_5X5_1000) # Aruco dictionary
    
    # To calculate the absolute value or modulus of a point
    def Absolute_Value(self, A):
        return math.sqrt(A[0]**2 + A[1]**2)

    # To calculate euclidean distance between two points
    def Euclidean_Distance(self, A, B):
        return math.sqrt(((B[0] - A[0])**2) + ((B[1] - A[1])**2))

    # To calculate the perpendicular distance from a point to a line
    def Perpendicular_Distance(self, Line, Point):
        return abs(Line[0] * Point[0] + Line[1] * Point[1] + Line[2]) / self.Absolute_Value(Line[:2]) 

    # To calculate the point of intersection of the perpendicular line drawn from a point to a line
    def Point_of_Intersection(self, Line, Point):
        x = ((Line[1] * (Line[1] * Point[0] - Line[0] * Point[1])) - (Line[0] * Line[2])) / (Line[0]**2 + Line[1]**2)
        y = ((Line[0] * (Line[0] * Point[1] - Line[1] * Point[0])) - (Line[1] * Line[2])) / (Line[0]**2 + Line[1]**2)
        return x, y

    # To initialize the left right limits for zig-zag motion as per drone position
    # It calculates the equation of lines at left extremum and right extremum at a given distance from drone position
    def Initialize_Limits(self, yaw, pos):
        self.Limits_Initialized = True

        x1, y1 = self.World_Pos(yaw, pos, [320, 0])
        x2, y2 = self.World_Pos(yaw, pos, [320, 480])

        self.Front = [0,0,0]
        self.Current_Front_Distance = 0
        self.Switch = True

        a = y2 - y1
        b = x1 - x2
        c = x2 * y1 - y2 * x1
        
        x1, y1 = self.World_Pos(yaw, pos, [0, 240])
        x2, y2 = self.World_Pos(yaw, pos, [640, 240])

        c1 = c + self.Left_Right_Limit * self.Absolute_Value([a,b]) 
        c2 = c - self.Left_Right_Limit * self.Absolute_Value([a,b])

        if self.Perpendicular_Distance([a,b,c1],[x1,y1]) < self.Perpendicular_Distance([a,b,c2],[x1,y1]):
            self.Left = [a,b,c1]
            self.Right = [a,b,c2]
        else:
            self.Left = [a,b,c2]
            self.Right = [a,b,c1]

        self.Last_Move = 'L'

    # To detect aruco and to get their centres and corners
    def Aruco(self, yaw, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray,150,255,cv2.THRESH_BINARY)
        corners, ids, _ = aruco.detectMarkers(thresh, self.Aruco_Dict, parameters=aruco.DetectorParameters_create())
 
        Centres = []
        Limits = []
        for i, corner in enumerate(corners):
            x = int((corner[0][0][0] + corner[0][2][0]) / 2)
            y = int((corner[0][0][1] + corner[0][2][1]) / 2)

            Limits.append(corner[0].tolist())

            if ids[i][0] == 0:
                self.Landing = True
                self.Aruco_Position = self.World_Pos(yaw, pos, [x,y])
                return None, None

            Centres.append([x,y])

        return Centres, Limits

    # To detect white point and to get their centres
    def White_Points(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray,150,255,cv2.THRESH_BINARY)
    
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_DILATE, kernel, iterations = 5)

        contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) # For Windows
        # _, contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) # For Linux

        Centres = []
        for contour in contours:
            if cv2.contourArea(contour) > 150:
                M = cv2.moments(contour)
                x = int(M['m10']/M['m00'])
                y = int(M['m01']/M['m00'])
                    
                Centres.append([x,y])
        
        return Centres

    # To convert the image co-ordinates into the real world co-ordinate system
    def World_Pos(self, yaw, pos, centre):
        del_x = centre[0] - 320
        del_y = 240 - centre[1]
    
        Length = self.Euclidean_Distance(centre, (320, 240))

        img_angle1 = math.acos(del_y / Length)
        img_angle2 = math.asin(del_x / Length)
        
        if del_x <= 0 and del_y <= 0:
            img_angle = img_angle1 + math.pi
        elif del_x <= 0 and del_y >= 0:
            img_angle = img_angle2
        else:
            img_angle = img_angle1

        actual_angle = img_angle + yaw + math.pi
        
        fac_x = abs(math.cos(actual_angle))
        fac_y = abs(math.sin(actual_angle))

        if del_x < 0:
            fac_y *= -1
        if del_y > 0:
            fac_x *= -1            
        
        factor = self.Factor * pos[2]
        real_length = Length * factor
    
        real_x = real_length * fac_x + pos[0]
        real_y = real_length * fac_y + pos[1]
    
        return real_x, real_y

    # To follow zig-zag motion while no optimal point is available to visit
    # |------------<|
    # |>-----------^|
    # |^-----------<|
    # |      D-----^|
    # It moves drone from left extremum to right extremum and vice versa
    # It also makes drone traverse in forward direction with an optimal distance
    def No_Point(self, yaw, pos):
        left_distance = self.Perpendicular_Distance(self.Left, pos)
        right_distance = self.Perpendicular_Distance(self.Right, pos)

        if (self.Last_Move == 'R' and left_distance < self.Threshold_Distance) or (self.Last_Move == 'L' and right_distance < self.Threshold_Distance) or self.Current_Front_Distance > self.Threshold_Distance:
            if self.Switch:
                if self.Last_Move == 'L':
                    self.Last_Move = 'R'
                elif self.Last_Move == 'R':
                    self.Last_Move = 'L'

                x1, y1 = self.World_Pos(yaw, pos, [0, 240])
                x2, y2 = self.World_Pos(yaw, pos, [640, 240])

                a = y2 - y1
                b = x1 - x2
                c = x2 * y1 - y2 * x1

                c1 = c + self.Front_Limit * self.Absolute_Value([a,b]) 
                c2 = c - self.Front_Limit * self.Absolute_Value([a,b])

                x, y = self.World_Pos(yaw, pos, [320, 0])

                if self.Perpendicular_Distance([a,b,c1],[x,y]) < self.Perpendicular_Distance([a,b,c2],[x,y]):
                    self.Front = [a,b,c1]   
                else:
                    self.Front = [a,b,c2]
                
                self.Switch = False

            x, y = self.Point_of_Intersection(self.Front, pos)
            self.Current_Front_Distance = self.Perpendicular_Distance(self.Front, pos)    

        elif self.Last_Move == 'L':
            x, y = self.Point_of_Intersection(self.Right, pos)
            self.Switch = True
        elif self.Last_Move == 'R':
            x, y = self.Point_of_Intersection(self.Left, pos)
            self.Switch = True
        
        return [x, y, self.Drone_Height]

    # It combines everything on priority basis and returns the setpoints where drone has to go
    # First it searches for aruco, if correct aruco is found we land, if not we visit nearest incorrect aruco and marks it visited
    # If no aruco is found we search for white points, if found and is unvisited, we visit it
    # If nothing is found in frame or no optimal point is available to visit we follow zig-zag motion until it finds an optimal point to visit
    # Here optimal point means something that is still unvisited 
    def Main(self, yaw, pos, img):
        if self.Landed:
            return None

        if not self.Landing:
            Centres, Limits = self.Aruco(yaw, img)
        
        if self.Landing:
            x, y = self.Aruco_Position
            if self.Euclidean_Distance([x,y], pos) > 0.5:
                return [x, y, self.Drone_Height]
            elif self.Euclidean_Distance([x,y], pos) > self.Threshold_Distance:
                return [x, y, 2.0]
            else:
                self.Landed = True
                return [x, y, 0.0]
        
        Unvisited = []
        for Centre, Limit in zip(Centres, Limits):
            world_pos = self.World_Pos(yaw, pos, Centre)
            Flag = False
            for P in self.Visited_Aruco:
                if self.Euclidean_Distance(P, world_pos) < self.Aruco_Length:
                    Flag = True
                    break
            if not Flag:
                List = [self.Euclidean_Distance(pos, world_pos), world_pos]
                for Position in Limit:
                    List.append(self.World_Pos(yaw, pos, Position))
                Unvisited.append(List)
        
        if len(Unvisited):
            Unvisited = sorted(Unvisited)
            x, y = Unvisited[0][1]
 
            if Unvisited[0][0] < self.Threshold_Distance:
                self.Visited_Aruco.extend(Unvisited[0][1:])
            if not self.Limits_Initialized:
                self.Initialize_Limits(yaw, pos)

            return [x, y, self.Drone_Height]

        Centres = self.White_Points(img)
    
        Unvisited = []
        for Centre in Centres:
            world_pos = self.World_Pos(yaw, pos, Centre)
            Flag = False
            for P in self.Visited_Aruco:
                if self.Euclidean_Distance(P, world_pos) < self.Aruco_Length:
                    Flag = True
                    break
            if not Flag:
                Unvisited.append([self.Euclidean_Distance(pos, world_pos), world_pos])
    
        if len(Unvisited):
            Unvisited = sorted(Unvisited)
            x, y = Unvisited[0][1]

            if not self.Limits_Initialized:
                self.Initialize_Limits(yaw, pos)

            return [x, y, self.Drone_Height]
        
        if self.Limits_Initialized:
            return self.No_Point(yaw, pos)
       
if __name__ == "__main__":
    cont = Controller()
    while(pos==None):
        continue
        
    ar = Aruco_Land()
    cont.aruco = ar
    # Flight variables
    takeoff_height = 3
    velocity = 0.6

    #cont.connect()
    #cont.takeoff(takeoff_height)

    #input_thread = threading.Thread(target=take_inputs, args=(velocity, cont,ar))
    #input_thread.start()

    rate = rospy.Rate(10)

    try:
        while not rospy.is_shutdown():
            rate.sleep()
    except KeyboardInterrupt:
        rospy.loginfo("Manual Interruption Occured")
        KILL_THREAD = True
        cv2.destroyAllWindows()
        #input_thread.join()

    cv2.destroyAllWindows()
