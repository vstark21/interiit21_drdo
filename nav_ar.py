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

pi_2 = math.pi / 2.0
global KILL_THREAD
KILL_THREAD = False

img=None
pos=None
r=None
p=None
y=None
class Controller:

    def __init__(self):
        rospy.init_node('control_node')
        rospy.Subscriber("/mavros/state", State, self.state_callback)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pos_callback)
        # Better to comment these lines, unless you need them
        # rospy.Subscriber("/depth_camera/rgb/image_raw", Image, self.dpcamrgb_callback)
        # rospy.Subscriber("/depth_camera/depth/image_raw", Image, self.dpcam_callback)
        rospy.Subscriber("/camera/color/image_raw", Image, self.downcam_callback)
        self.cmd_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)

        self.mode_service = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

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
            
           
            cv2.imshow("Downward_rgb", image3)
            cv2.waitKey(1)
        except Exception as e:
            rospy.loginfo(e)

    
    def goto(self, pose):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.timestamp
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

    def set_vel(self, vx, vy, vz, avx=0, avy=0, avz=0):

        vel = Twist()

        vel.linear.x = vx
        vel.linear.y = vy
        vel.linear.z = vz

        vel.angular.x = avx
        vel.angular.y = avy
        vel.angular.z = avz

        self.cmd_vel_pub.publish(vel)

    def moveF(self, vel):
        quats = [self.pose.orientation.w,
                self.pose.orientation.x,
                self.pose.orientation.y,
                self.pose.orientation.z]
        theta, _, _ = tf.transformations.euler_from_quaternion(quats)
        vx = -vel * math.cos(theta)
        vy = vel * math.sin(theta)
        self.set_vel(vx, vy, 0)

    def moveB(self, vel):
        quats = [self.pose.orientation.w,
                self.pose.orientation.x,
                self.pose.orientation.y,
                self.pose.orientation.z]
        theta, _, _ = tf.transformations.euler_from_quaternion(quats)
        vx = vel * math.cos(theta)
        vy = -vel * math.sin(theta)
        self.set_vel(vx, vy, 0)

    def moveL(self, vel):
        quats = [self.pose.orientation.w,
                self.pose.orientation.x,
                self.pose.orientation.y,
                self.pose.orientation.z]
        theta, _, _ = tf.transformations.euler_from_quaternion(quats)
        vx = -vel * math.sin(theta)
        vy = -vel * math.cos(theta)
        self.set_vel(vx, vy, 0)

    def moveR(self, vel):
        quats = [self.pose.orientation.w,
                self.pose.orientation.x,
                self.pose.orientation.y,
                self.pose.orientation.z]
        theta, _, _ = tf.transformations.euler_from_quaternion(quats)
        vx = vel * math.sin(theta)
        vy = vel * math.cos(theta)
        self.set_vel(vx, vy, 0)

    # Counter clockwise
    def rotateCC(self, vel):
        self.set_vel(0, 0, 0, avz=vel)

    # Clockwise
    def rotateC(self, vel):
        self.set_vel(0, 0, 0, avz=-vel)

    def stop(self):
        self.set_vel(0, 0, 0)

    def arm(self):

        return self.arm_service(True)
        
    def disarm(self):

        return self.arm_service(False)

    def takeoff(self, height):

        mode_resp = self.mode_service(custom_mode="4")
        self.arm()

        takeoff_resp = self.takeoff_service(altitude=height)
        return takeoff_resp
    
    def land(self):

        land_resp = self.mode_service(custom_mode="9")
        self.disarm()
    
    def connect(self):

        vel = Twist()
        for i in range(100):
            self.cmd_vel_pub.publish(vel)
        
        while not self.state.connected:
            rospy.loginfo_once("Connecting to drone")
            rospy.sleep(0.01)
        
        rospy.loginfo("Connected to Drone")

        while self.state.mode != "GUIDED":
            rospy.loginfo_once("Waiting for mode to set to `GUIDED`")
            rospy.sleep(0.01)
        
        rospy.loginfo("Mode set to `GUIDED`")

class Aruco_land():
    def __init__(self):
        
        optimal_length = 100 # Need To Be Adjust
        self.Square = [(pos[0] - optimal_length,pos[1]),
                (pos[0] + optimal_length,pos[1]),
                (pos[0] - optimal_length,pos[1] + optimal_length),
                (pos[0] + optimal_length,pos[1] + optimal_length)]
        
        self.Visited = []
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_1000)



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
            return (cx, cy, pos[2]), False
        
        return self.No_Point()
        
def take_inputs(velocity, controller,aruco):
    msg = """
        Reading from the keyboard  and Publishing to Drone!
        ---------------------------
        Moving around:
                 w
        z    a       d   x
                 s
        
        w : Move Forward
        s : Move Backward
        a : Move Left
        d : Move Right
        z : rotate counter-clockwise
        x : rotate clockwise
        space : STOP
        CTRL-C to quit
        """
    print msg
    settings = termios.tcgetattr(sys.stdin)

    def getKey(key_timeout):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    while not rospy.is_shutdown() and not KILL_THREAD:
        x = getKey(0.1)
        if x == "w" or x == "W":
            controller.moveF(velocity)
        elif x == "s" or x == "S":
            controller.moveB(velocity)
        elif x == "a" or x == "A":
            controller.moveL(velocity)
        elif x == "d" or x == "D":
            controller.moveR(velocity)
        elif x == "z" or x == "Z":
            controller.rotateCC(velocity)
        elif x == "x" or x == "X":
            controller.rotateC(velocity)
        elif x == " ":
            controller.stop()
        elif x == '\x03':
            break

        # cv2.imshow("img",controller.down_cam)
        # if cv2.waitKey(1) & 0xFF==ord('q'):
        #     break
        
       
       
        
        data=aruco.Main(controller.down_cam,pos,y)
        print(data)
        # controller.goto_xyz_rpy( 0, 0, 1, 0, 0, 0)


if __name__ == "__main__":

    cont = Controller()
    while(pos==None):
        continue

    ar=Aruco_land()

    # Flight variables
    takeoff_height = 3
    velocity = 0.6

    cont.connect()
    cont.takeoff(takeoff_height)

    input_thread = threading.Thread(target=take_inputs, args=(velocity, cont,ar))
    input_thread.start()

    rate = rospy.Rate(10)

    try:
        while not rospy.is_shutdown():
            rate.sleep()
    except KeyboardInterrupt:
        rospy.loginfo("Manual Interruption Occured")
        KILL_THREAD = True
        cv2.destroyAllWindows()
        input_thread.join()

    cv2.destroyAllWindows()