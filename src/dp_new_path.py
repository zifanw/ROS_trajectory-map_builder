#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D

BUFFER_MAX = 1000

class Message():
    def __init__(self):
        self.header = []
        self.length = []
        self.chanNum = []
        self.packageID = []
        self.status = []
        self.data = []
        self.Checksum = []

class dp():
    def __init__(self):
	    rospy.init_node('dp', anonymous=True)
	    rospy.loginfo('The infocenter is initilized...')
	    self.pub = rospy.Publisher('/info/trajectory', Pose2D, queue_size = 100)
	    rospy.wait_for_message('read',String)
	    rospy.Subscriber('read',String, self.callback, queue_size=1)
            self.buffer = []

    def callback(self, data):
        #self.postdata = data.data.split(',')
        #rospy.loginfo(self.postdata)
        L = list(data.data)
        rospy.loginfo("read from wifi: ")
        rospy.loginfo(L)
        L = map(ord, L)
        #rospy.loginfo(self.buffer[0])
        if 0x02 in L:
            pose_id = L.index(2)
            if L[pose_id-3]==0xA5 and L[pose-2]==0x5A:
                x_low = L[pose_id+3]
                x_high = L[pose_id+4]
                y_low = L[pose_id+5]
                y_high = L[pose_id+6]
                theta_low = L[pose_id+7]
                theta_high = L[pose_id+8]
                self.process_a_pose(x_low, x_high, y_low, y_high, theta_low, theta_high)

    def process_a_pose(self, x_low, x_high, y_low, y_high, theta_low, theta_high):
        pose = Pose2D()
        pose.x = self.hex2speed(x_low, x_high) # [4] is LOW byte and [5] is the HIGH byte
        pose.y = self.hex2speed(y_low, y_high)
        pose.theta = self.hex2rad(theta_low, theta_high)
        rospy.loginfo("x: "+str(pose.x)+" y: " +str(pose.y)+" theta: "+str(pose.theta))
        self.pub.publish(pose)

    def hex2speed(self, low, high):
        speed = low + (high << 8)
        if speed > 32768:
            speed = ~((~speed) & 0x0000FFFF)
        return float(speed) / 100.0

    def hex2rad(self, low, high):
        rad = low + (high << 8)
        if rad > 32768:
            rad = ~(~(rad) & 0x0000FFFF)
        return float(rad) / 100.0


    def string2msg_pose(self, streamin):
        #streamin = map(self.Positive, streamin)
        msg = Message()
        msg.header.extend(streamin[0:2])
        msg.length.append(streamin[2])
        msg.chanNum.append(streamin[3])
        msg.packageID.append(streamin[4])
        msg.status.append(streamin[5])
        msg.data.extend(streamin[6:31])
        msg.Checksum.append(streamin[31])
        return msg

    def string2msg_map(self, streamin):
        msg = Message()
        msg.header.extend(streamin[0:2])
        msg.length.append(streamin[2])
        msg.chanNum.append(streamin[3])
        msg.packageID.append(streamin[4])
        msg.data.extend(streamin[5:31])
        return msg


    def checksum(self, msg):
        Sum = msg.length[0] + msg.chanNum[0] + msg.packageID[0] + msg.status[0] + sum(msg.data)
        Sum = Sum & 0xFF
        if Sum == msg.Checksum[0]:
            return True
        else:
            return False


if __name__ == '__main__':
    try:
    	dp()
    	rospy.spin()
    except:
    	pass
