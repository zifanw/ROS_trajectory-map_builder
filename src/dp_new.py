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
        L = map(ord, L)
        self.buffer.extend(L)
        #rospy.loginfo(self.buffer)
        rospy.loginfo("the buffer length is: "+str(len(self.buffer)))
        #rospy.loginfo(self.buffer[0])
        head = self.buffer.index(0xA5)
        if self.buffer[head+1] == 0x5A:
            length = self.buffer[head+2]
            #rospy.loginfo(length)
            package = self.buffer[head:head+length+3]
            self.buffer = self.buffer[head+2+length:] # discard the processed package
            #rospy.loginfo(self.buffer[0:100])
            #rospy.loginfo(self.msg.header)
            #rospy.loginfo(package)
            #rospy.loginfo(len(package))
            msg = Message()
            if package[3] == 0x02:
                msg = self.string2msg_pose(package)
                rospy.loginfo("this is a pose message")
                self.process_a_pose(msg)
            elif package[3] == 0x03:
                self.msg = self.string2msg_map(package)
                rospy.loginfo("this is a map message")
                rospy.loginfo(package)

        elif self.buffer[head + 2] == 0x5A:
            head += 1
            length = self.buffer[head+2]
            #rospy.loginfo(length)
            package = self.buffer[head:head+length+3]
            self.buffer = self.buffer[head+2+length:] # discard the processed package
            #rospy.loginfo(self.buffer[0:100])
            #rospy.loginfo(self.msg.header)
            rospy.loginfo(package)
            rospy.loginfo(len(package))
            msg = Message()
            if package[3] == 0x02:
                msg = self.string2msg_pose(package)
                rospy.loginfo("this is a pose message")
                self.process_a_pose(msg)
            elif package[3] == 0x03:
                self.msg = self.string2msg_map(package)
                rospy.loginfo("this is a map message")
        else:
            rospy.loginfo("cannot find a valid header")
            #rospy.loginfo(self.buffer[head:200])



    def process_a_pose(self, msg):
        if self.checksum(msg):
            #rospy.loginfo("this is a pose message")
            pose = Pose2D()
            pose.x = self.hex2speed(msg.data[0], msg.data[1]) # [4] is LOW byte and [5] is the HIGH byte
            pose.y = self.hex2speed(msg.data[2], msg.data[3])
            pose.theta = self.hex2rad(msg.data[4], msg.data[5])
            rospy.loginfo("x: "+str(pose.x)+" y: " +str(pose.y)+" theta: "+str(pose.theta))
            self.pub.publish(pose)
        else:
            rospy.loginfo("checksum error")
        #else:
            #rospy.loginfo("waiting for a complete package")

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
