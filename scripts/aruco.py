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
       
        try:
            bridge = CvBridge()
            image3 = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')[:, :, ::-1]
            
            
            
            self.down_cam=image3
            if self.aruco is not None:
                global y,pos
                print(self.aruco.Main(self.down_cam,[ self.pose.position.x, self.pose.position.y, self.pose.position.z],y))
            cv2.imshow("Downward_rgb", image3)
            cv2.waitKey(1)
        except Exception as e:
            rospy.loginfo(e)

class Aruco_land():
    def __init__(self):
        self.Flag = False
        self.Landed = False
        self.Visited_Aruco = []
        self.Visited_Points = []
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_1000)
    
    def Initialize_Limits(self, yaw, pos):
        self.Flag = True

        optimal_length = 5
        Array = np.array(pos[:2]) - np.array(self.World_Pos(yaw, pos, [0, 240]))
        index = np.argmax(abs(Array))

        self.Left =  [index, pos[index] - (int(Array[index]/abs(Array[index])) * optimal_length)]
        self.Right = [index, pos[index] + (int(Array[index]/abs(Array[index])) * optimal_length)]
        
        Array = np.array(self.World_Pos(yaw, pos, [320, 0])) - np.array(pos[:2])
        index = np.argmax(abs(Array))

        self.Front = [index, int(Array[index]/abs(Array[index])) * optimal_length]
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

        contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        Centres = []
        for contour in contours:
            if cv2.contourArea(contour) > 150:
                M = cv2.moments(contour)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                    
                Centres.append([cx,cy])
        
        return Centres

    def Distance(self, A, B):
        return math.sqrt(((B[0] - A[0])**2) + ((B[1] - A[1])**2))

    def World_Pos(self, yaw, pos, centre):
        del_x = centre[0] - 320
        del_y = 240 - centre[1]
    
        Length = self.Distance(centre, (320, 240))

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

    def No_Point(self, pos):
        left_distance = abs(pos[self.Left[0]] - self.Left[1])
        right_distance = abs(pos[self.Right[0]] - self.Right[1])

        if left_distance < 0.3 or right_distance < 0.3:
            if self.last_move == 'L':
                self.last_move = 'R'
            elif self.last_move == 'R':
                self.last_move = 'L'
            
            Point = pos[:2][:]
            Point[self.Front[0]] += self.Front[1]

            return [Point[0], Point[1], 3.0]
        elif self.last_move == 'L':
            Point = pos[:2][:]
            Point[self.Right[0]] = self.Right[1]

            return [Point[0], Point[1], 3.0]
        elif self.last_move == 'R':
            Point = pos[:2][:]
            Point[self.Left[0]] = self.Left[1]

            return [Point[0], Point[1], 3.0]

    def Main(self, img, pos, yaw):
        if self.Landed:
            return None, True

        Centres, Flag = self.Aruco(img)
    
        if Flag:
            cx, cy = self.World_Pos(yaw, pos, Centres[0])
            if self.Distance([cx,cy], pos[:2]) < 0.6 and pos[2] > 2.6:
                return [cx, cy, 2.0], False
            elif self.Distance([cx,cy], pos[:2]) < 0.35 and pos[2] > 1.6:
                self.Landed = True
                return [cx, cy, 0.23], True
        
        Unvisited = []
        for Centre in Centres:
            world_pos = self.World_Pos(yaw, pos, Centre)
            Flag = False
            for P in self.Visited_Aruco:
                if self.Distance(P, world_pos) < 0.6:
                    Flag = True
                    break
            if not Flag:
                Unvisited.append([self.Distance(pos, world_pos), world_pos])
        
        if len(Unvisited):
            Unvisited = sorted(Unvisited)
            cx, cy = Unvisited[0][1]
            self.Visited_Points.append([int(cx),int(cy)])
            
            if Unvisited[0][0] < 0.6:
                self.Visited_Aruco.append([cx,cy])
            if not self.Flag:
                self.Initialize_Limits(yaw, pos)

            return [cx, cy, 3.0], False

        Centres = self.White_Points(img)
    
        Unvisited = []
        for Centre in Centres:
            world_pos = self.World_Pos(yaw, pos, Centre)
            Flag = False
            for P in self.Visited_Aruco:
                if self.Distance(P, world_pos) < 1.1:
                    Flag = True
                    break
            if not Flag:
                Unvisited.append([self.Distance(pos, world_pos), world_pos])
    
        if len(Unvisited):
            Unvisited = sorted(Unvisited)
            cx, cy = Unvisited[0][1]
            self.Visited_Points.append([int(cx),int(cy)])

            if not self.Flag:
                self.Initialize_Limits(yaw, pos)

            return [cx, cy, 3.0], False
        
        if self.Flag:
            return self.No_Point(pos), False
        
        return None, False

if __name__ == "__main__":
    #ar=Aruco_land()
    cont = Controller()
    while(pos==None):
        continue

    ar=Aruco_land()
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
