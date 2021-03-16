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

pi_2 = math.pi / 2.0
global KILL_THREAD
KILL_THREAD = False

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

    def state_callback(self, data):
        self.state = data

    def pos_callback(self, data):
        self.timestamp = data.header.stamp
        self.pose = data.pose
    
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
    
def take_inputs(velocity, controller):
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


if __name__ == "__main__":

    cont = Controller()

    # Flight variables
    takeoff_height = 5
    velocity = 0.3

    cont.connect()
    cont.takeoff(takeoff_height)

    input_thread = threading.Thread(target=take_inputs, args=(velocity, cont))
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
