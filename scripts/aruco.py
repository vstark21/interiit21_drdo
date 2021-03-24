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
timestamp = rospy.Time()
bridge = CvBridge()
down_cam=np.zeros((640,400,3),np.uint8)

def pos_callback(data):
    timestamp = data.header.stamp
    pose = data.pose
    global pos
              
    pos=[ data.pose.position.x, data.pose.position.y, data.pose.position.z]
    global r,p,y
    quats = [pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z]
    r,p,y = tf.transformations.euler_from_quaternion(quats)

def downcam_callback(data):
       
    try:
        bridge = CvBridge()
        image3 = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')[:, :, ::-1]
            
            
            
        down_cam=image3
        global ar
        if ar is not None:
            global y,pos,pose_pub
            data = ar.Main(y, pos, down_cam)
            setpoints = Setpoints()
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = data
            setpoints.setpoints = [pose]
            setpoints.header.stamp = rospy.get_rostime()
            setpoints.header.frame_id= "map"
            pose_pub.publish(setpoints)

    except Exception as e:
        rospy.loginfo(e)


class Aruco_Land():
    def __init__(self):
        self.Flag = False
        self.Landed = False
        self.Visited_Aruco = []
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_1000)
    
    def Absolute_Value(self, A):
        return math.sqrt(A[0]**2 + A[1]**2)

    def Perpendicular_Distance(self, Line, Point):
        return abs(Line[0] * Point[0] + Line[1] * Point[1] + Line[2]) / self.Absolute_Value(Line[:2]) 

    def Point_of_Intersection(self, Line, Point):
        x = ((Line[1] * (Line[1] * Point[0] - Line[0] * Point[1])) - (Line[0] * Line[2])) / (Line[0]**2 + Line[1]**2)
        y = ((Line[0] * (Line[0] * Point[1] - Line[1] * Point[0])) - (Line[1] * Line[2])) / (Line[0]**2 + Line[1]**2)
        return x, y

    def Initialize_Limits(self, yaw, pos):
        self.Flag = True

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

        c1 = c + 3.0 * self.Absolute_Value([a,b]) 
        c2 = c - 3.0 * self.Absolute_Value([a,b])

        if self.Perpendicular_Distance([a,b,c1],[x1,y1]) < self.Perpendicular_Distance([a,b,c2],[x1,y1]):
            self.Left = [a,b,c1]
            self.Right = [a,b,c2]
        else:
            self.Left = [a,b,c2]
            self.Right = [a,b,c1]

        self.last_move = 'L'

    def Aruco(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray,150,255,cv2.THRESH_BINARY)
        corners, ids, _ = aruco.detectMarkers(thresh, self.aruco_dict, parameters=aruco.DetectorParameters_create())
 
        Centres = []
        for i, corner in enumerate(corners):
            x = int((corner[0][0][0] + corner[0][2][0]) / 2)
            y = int((corner[0][0][1] + corner[0][2][1]) / 2)
            
            if ids[i][0] == 0:
                return [[x,y]], True

            Centres.append([x,y])

        return Centres, False

    def White_Points(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray,150,255,cv2.THRESH_BINARY)
    
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_DILATE, kernel, iterations = 5)

        # contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) # For Windows
        _, contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) # For Linux

        Centres = []
        for contour in contours:
            if cv2.contourArea(contour) > 150:
                M = cv2.moments(contour)
                x = int(M['m10']/M['m00'])
                y = int(M['m01']/M['m00'])
                    
                Centres.append([x,y])
        
        return Centres

    def Euclidean_Distance(self, A, B):
        return math.sqrt(((B[0] - A[0])**2) + ((B[1] - A[1])**2))

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
        
        factor = 0.003 * pos[2]
        real_length = Length * factor
    
        real_x = real_length * fac_x + pos[0]
        real_y = real_length * fac_y + pos[1]
    
        return real_x, real_y

    def No_Point(self, yaw, pos):
        left_distance = self.Perpendicular_Distance(self.Left, pos)
        right_distance = self.Perpendicular_Distance(self.Right, pos)

        if (self.last_move == 'R' and left_distance < 0.3) or (self.last_move == 'L' and right_distance < 0.3) or self.Current_Front_Distance > 0.3:
            if self.Switch:
                if self.last_move == 'L':
                    self.last_move = 'R'
                elif self.last_move == 'R':
                    self.last_move = 'L'

                x1, y1 = self.World_Pos(yaw, pos, [0, 240])
                x2, y2 = self.World_Pos(yaw, pos, [640, 240])

                a = y2 - y1
                b = x1 - x2
                c = x2 * y1 - y2 * x1

                c1 = c + 2.3 * self.Absolute_Value([a,b]) 
                c2 = c - 2.3 * self.Absolute_Value([a,b])

                x, y = self.World_Pos(yaw, pos, [320, 0])

                if self.Perpendicular_Distance([a,b,c1],[x,y]) < self.Perpendicular_Distance([a,b,c2],[x,y]):
                    self.Front = [a,b,c1]   
                else:
                    self.Front = [a,b,c2]
                
                self.Switch = False

            x, y = self.Point_of_Intersection(self.Front, pos)
            self.Current_Front_Distance = self.Perpendicular_Distance(self.Front, pos)    

        elif self.last_move == 'L':
            x, y = self.Point_of_Intersection(self.Right, pos)
            self.Switch = True
        elif self.last_move == 'R':
            x, y = self.Point_of_Intersection(self.Left, pos)
            self.Switch = True
        
        return [x, y, 3.0]

    def Main(self, yaw, pos, img):
        if self.Landed:
            return None

        Centres, Flag = self.Aruco(img)
    
        if Flag:
            x, y = self.World_Pos(yaw, pos, Centres[0])
            if self.Euclidean_Distance([x,y], pos) < 0.6 and pos[2] > 2.6:
                return [x, y, 2.0]
            elif self.Euclidean_Distance([x,y], pos) < 0.35 and pos[2] > 1.6:
                self.Landed = True
                return [x, y, 0.15]
        
        Unvisited = []
        for Centre in Centres:
            world_pos = self.World_Pos(yaw, pos, Centre)
            Flag = False
            for P in self.Visited_Aruco:
                if self.Euclidean_Distance(P, world_pos) < 2.2:
                    Flag = True
                    break
            if not Flag:
                Unvisited.append([self.Euclidean_Distance(pos, world_pos), world_pos])
        
        if len(Unvisited):
            Unvisited = sorted(Unvisited)
            x, y = Unvisited[0][1]
 
            if Unvisited[0][0] < 0.6:
                self.Visited_Aruco.append([x,y])
            if not self.Flag:
                self.Initialize_Limits(yaw, pos)

            return [x, y, 3.0]

        Centres = self.White_Points(img)
    
        Unvisited = []
        for Centre in Centres:
            world_pos = self.World_Pos(yaw, pos, Centre)
            Flag = False
            for P in self.Visited_Aruco:
                if self.Euclidean_Distance(P, world_pos) < 2.2:
                    Flag = True
                    break
            if not Flag:
                Unvisited.append([self.Euclidean_Distance(pos, world_pos), world_pos])
    
        if len(Unvisited):
            Unvisited = sorted(Unvisited)
            x, y = Unvisited[0][1]

            if not self.Flag:
                self.Initialize_Limits(yaw, pos)

            return [x, y, 3.0]
        
        if self.Flag:
            return self.No_Point(yaw, pos)

       
if __name__ == "__main__":
    ar = Aruco_Land()
    rospy.init_node('camera_node')
    #rospy.Subscriber("/mavros/state", State, state_callback)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pos_callback)
        # Better to comment these lines, unless you need them
        # rospy.Subscriber("/depth_camera/rgb/image_raw", Image, self.dpcamrgb_callback)
        # rospy.Subscriber("/depth_camera/depth/image_raw", Image, self.dpcam_callback)

    rospy.Subscriber("/camera/color/image_raw", Image, downcam_callback)
    pose_pub = rospy.Publisher("/setpoint_array", Setpoints, queue_size=1)
    while(pos==None):
        continue
        
    

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
