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
        
        optimal_length = 100 # Need To Be Adjust
        self.Square = [(pos[0] - optimal_length,pos[1]),
                (pos[0] + optimal_length,pos[1]),
                (pos[0] - optimal_length,pos[1] + optimal_length),
                (pos[0] + optimal_length,pos[1] + optimal_length)]
        
        self.Visited = []
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_1000)
        self.set_control = rospy.Publisher("/setpoint_array",Setpoints,queue_size=1)


    def Aruco(self,img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        corners, ids, _ = aruco.detectMarkers(thresh, self.aruco_dict, parameters=aruco.DetectorParameters_create())
        
        Centres = []
        for i, corner in enumerate(corners):
            x = int((corner[0][0][0] + corner[0][2][0]) / 2)
            y = int((corner[0][0][1] + corner[0][2][1]) / 2)
            
            if ids[i][0] == 0:
                return [(x,y)], True
            
            Centres.append((x,y))
        
        return Centres, False

    def White_Points(self,img):
        mask = cv2.inRange(img, np.array([100,100,100]), np.array([255,255,255]))
        img = cv2.bitwise_and(img, img, mask = mask)
        
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        blur = cv2.GaussianBlur(gray,(5,5),0)
        _, thresh = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CROSS, kernel, iterations = 5)

        _,contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        Centres = []
        for contour in contours:
            if cv2.contourArea(contour) > 150:
                _, _, w, h = cv2.boundingRect(contour)
                if w/h > 0.5:
                    M = cv2.moments(contour)
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    
                    Centres.append((cx,cy))
                    
        return Centres

    def Distance(self,A,B):
        return ((B[0] - A[0])**2 + (B[1] - A[1])**2)**(1/2)

    def World_Pos(self,pitch, pos, centre):
        del_x = centre[0] - 480
        del_y = centre[1] - 640
        Complex = complex(math.cos(pitch), math.sin(pitch)) * complex(del_x, del_y)
        k = 100 ## Need to be Adjusted
        x = pos[0] + Complex.real / (k * pos[2])
        y = pos[1] + Complex.imag / (k * pos[2])
        return x, y

    def No_Point(self):
        pass

    def Main(self,img, pos, pitch):
        Centres, Flag = self.Aruco(img)
        
        if Flag:
            cx, cy = self.World_Pos(pitch, pos, Centres[0])
            return (cx, cy, pos[2]), True
        
        for Centre in Centres:
            world_pos = self.World_Pos(pitch, pos, Centre)
            Flag = False
            for P in self.Visited:
                if self.Distance(P, world_pos) < 20:
                    Flag = True
                    break
            if not Flag:
                self.Visited.append(world_pos)
        
        Unvisited = []
        Centres = self.White_Points(img)
        
        for Centre in Centres:
            world_pos = self.World_Pos(pitch, pos, Centre)
            Flag = False
            for P in self.Visited:
                if self.Distance(P, world_pos) < 50:
                    Flag = True
                    break
            if not Flag:
                Unvisited.append([self.Distance(pos, world_pos), world_pos])
        
        if len(Unvisited):
            Unvisited = sorted(Unvisited)
            cx, cy = Unvisited[0][1]
            ## Wrong coordnates
        
            temp = Setpoints()
            a = Pose()
            a.position.x = cx
            a.position.y = cy
            a.position.z = pos[-1]
            temp.setpoints = [a]
            temp.header.stamp = rospy.Time.now()
        
            #self.set_control.publish(temp)
            return (cx, cy, pos[2]), False
        
        
        

        return self.No_Point()
        


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