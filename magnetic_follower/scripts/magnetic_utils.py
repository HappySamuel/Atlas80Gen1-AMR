import rospy
import numpy as np
import tf.transformations as tftr
import json

from geometry_msgs.msg import Vector3, Point, PoseWithCovarianceStamped, Twist, Quaternion


# Magnetic DATA (16 channel):
#
#    [0/1/2/3/4/5/6/7/8/9/10/11/12/13/14/15]
#      LEFT        MIDDLE           RIGHT
#
# Filter Method: (1) data > 120         -->   1
#                (2) data < -150        -->  -1
#                (3) -150 < data < 120  -->   0
#
# Wherever "1" is found in the data, it is counted as "middle".
# Then the data on the left channels which next to the "middle" are counted as "LEFT side".
# While the data on the right channels which next to the "middle" are counted as "RIGHT side".
#
# Direction "in"  -->  1
#           "out" -->  2
class MarkerIdentifier(object):
    def __init__(self):
#        self.mag_msg = mag_msg
        self.left_marker = False
        self.right_marker = False
        self.prev_left_marker = False
        self.prev_right_marker = False
        self.starter = True
        self.side_registered = 2
        self.marker_msg = None
        self.reset = False

    def identifier(self, mag_msg):
        # Use for resetting counter
        if(self.reset == True):
            self.left_marker = False
            self.right_marker = False
            self.prev_left_marker = False
            self.prev_right_marker = False
            self.starter = True
            self.side_registered = 2
            self.marker_msg = None
            self.reset = False
        s_count = []
        for i in range(0,len(mag_msg)):
            if(mag_msg[i] == 1):
                s_count.append(i)
        if(s_count != []):
            for i in range(0,len(mag_msg)):
                if(mag_msg[i] == -1 and i < s_count[0] and self.starter == True):
                    self.side_registered = 0    # 0 - Left marker as base
                    self.starter = False
                elif(mag_msg[i] == -1 and i > s_count[-1] and self.starter == True):
                    self.side_registered = 1    # 1 - Right marker as base
                    self.starter = False
            for i in range(0,s_count[0]):
                if(mag_msg[i] == -1):
                    self.left_marker = True
                    break
                else:
                    self.left_marker = False
            for i in range(s_count[-1],16):
                if(mag_msg[i] == -1):
                    self.right_marker = True
                    break
                else:
                    self.right_marker = False
            # Left marker as base, Right marker as counter
            if(self.starter == False and self.side_registered == 0):
                if(self.left_marker == True):    # When left marker is present
                    if(self.prev_left_marker == False):
                        self.marker_count = 0    # clear counter when left marker first appears
                    elif(self.right_marker == True and self.prev_right_marker == False):
                        self.marker_count += 1   # increase counter at every appearance of right marker
                elif(self.prev_left_marker == True): # left marker is absent but was present previously
                    if(self.marker_count == 1):
                        self.marker_count = None
                        self.marker_msg = "out-1"
                    elif(self.marker_count == 2):
                        self.marker_count = None
                        self.marker_msg = "out-2"
                    elif(self.marker_count == 3):
                        self.marker_count = None
                        self.marker_msg = "out-3"
                    elif(self.marker_count == 4):
                        self.marker_count = None
                        self.marker_msg = "out-4"
                    elif(self.marker_count == 5):
                        self.marker_count = None
                        self.marker_msg = "out-5"
                    self.starter = True
                self.prev_left_marker = self.left_marker
                self.prev_right_marker = self.right_marker
            # Right marker as base, Left marker as counter
            elif(self.starter == False and self.side_registered == 1):
                if(self.right_marker == True):    # When left marker is present
                    if(self.prev_right_marker == False):
                        self.marker_count = 0    # clear counter when left marker first appears
                    elif(self.left_marker == True and self.prev_left_marker == False):
                        self.marker_count += 1   # increase counter at every appearance of right marker
                elif(self.prev_right_marker == True): # left marker is absent but was present previously
                    if(self.marker_count == 0):
                        self.marker_count = None
                        self.marker_msg = "in-0"
                    elif(self.marker_count == 1):
                        self.marker_count = None
                        self.marker_msg = "in-1"
                    elif(self.marker_count == 2):
                        self.marker_count = None
                        self.marker_msg = "in-2"
                    elif(self.marker_count == 3):
                        self.marker_count = None
                        self.marker_msg = "in-3"
                    elif(self.marker_count == 4):
                        self.marker_count = None
                        self.marker_msg = "in-4"
                    elif(self.marker_count == 5):
                        self.marker_count = None
                        self.marker_msg = "in-5"
                    self.starter = True
                self.prev_left_marker = self.left_marker
                self.prev_right_marker = self.right_marker


# Convert Quaternion [q.x, q.y, q.z, q.w] into Yaw Angle [deg]
def quaternion_to_yaw(q):
    roll, pitch, yaw = np.rad2deg(tftr.euler_from_quaternion([q.x, q.y, q.z, q.w]))
    return yaw


# Convert Yaw Angle [deg] into Quaternion [q.x, q.y, q.z, q.w]
def yaw_to_quaternion(yaw):
    q = tftr.quaternion_from_euler(0.0, 0.0, np.deg2rad(yaw))
    return Quaternion(q[0], q[1], q[2], q[3])


# Distance Checker [m]
def dist_checker(a, b, tolerance):
    distance = np.sqrt((a.x - b.x)**2 + (a.y - b.y)**2)
    return True if(distance <= tolerance) else False


# Heading Checker [deg]
def heading_checker(a, b, tolerance):
    yaw_diff = a - b
    if(yaw_diff >= 180):
        yaw_diff = abs(yaw_diff - 360)
    elif(yaw_diff <= -180):
        yaw_diff = abs(yaw_diff + 360)
    return True if(yaw_diff <= tolerance) else False


# Distance and Heading Checker [m, m, deg]
def pose_checker(a, b, dist_tolerance, yaw_tolerance):
    return True if(dist_checker(a, b, dist_tolerance) and heading_checker(a.z, b.z, yaw_tolerance)) else False


# Generating "Relocalize Point" with "position" and "heading"
def relocalize_pt(pose):
    pt = PoseWithCovarianceStamped()
    pt.header.frame_id = "/map"
    pt.header.stamp = rospy.Time.now()
    pt.pose.pose.position = Point(pose.x, pose.y, 0.0)
    pt.pose.pose.orientation = yaw_to_quaternion(pose.z)
    return pt


# Generating "Drive Message" with "x_speed"
def drive_cmd(x_speed):
    drive_msg = Twist()
    drive_msg.linear = Vector3(x_speed, 0, 0)
    drive_msg.angular = Vector3(0, 0, 0)
    return drive_msg


class MagneticTrajectory(object):
    def __init__(self):
        self.code_action = []
    
    def load(self, path):
        with open(path) as json_file:
            json_data = json.load(json_file)
            for p in json_data["sequence"]:
                self.code_action.append((p["code"], p["action"]))

    def clear(self):
        self.code_action = []

