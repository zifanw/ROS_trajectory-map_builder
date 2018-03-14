#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
import time
from std_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import *

class dp():
    def __init__(self):
        rospy.init_node('map', anonymous=True)
        self.pub = rospy.Publisher('map', OccupancyGrid, queue_size = 1000)
        self.map = OccupancyGrid()
        self.map.header.seq = 1
        self.map.header.frame_id = 'odom'
        self.map.info.resolution = 0.05
        self.map.info.origin.position.z = 0
        self.map.info.origin.orientation.x = 0
        self.map.info.origin.orientation.y = 0
        self.map.info.origin.orientation.z = 0
        self.map.info.origin.orientation.w = 1
        self.map.info.origin.position.x = 0
        self.map.info.origin.position.y = 0
        self.map.info.width = 10
        self.map.info.height = 10
        self.buffer = []
        self.x = 0
        self.y = 0
        self.width = 1
        self.height = 1
        rospy.loginfo('The mapping is initialized and waiting for transmission...')
        rospy.Subscriber('/read/map',String, self.callback, queue_size=1000)
        rospy.wait_for_message('/read/map',String)

    def callback(self, data):
        L = list(data.data)
        #L = int(L)
        L = map(self.quatilization, map(ord, L))
        #rospy.loginfo(str(L))
        rospy.loginfo("len(L) = " + str(len(L)) + ", L[0] = " + str(L[0]) + ", L[1] = " + str(L[1]));
        if 0xA5 in L: # A5A5 is the head
            rospy.loginfo("the head is found.")
            
            if self.check(self.buffer):
                rospy.loginfo("A complete map is received and passed to the builder")
                self.mapping(self.buffer)
            else:
                rospy.loginfo("the last map is not valid")
            self.synch(L)

        elif 0x5A in L:
            head = L.index(0x5A)
            if L[head+1] == 0x5A:
                if head == 0:
                    self.buffer.extend(L[head+10:])
                else:
                    self.buffer.extend(L[0:head])
                    self.buffer.extend(L[head+10:])
            else:
                if 200 in L:
                    head = L.index(200)
                    if head == 0:
                        rospy.loginfo(str(L))
                    else:
                        self.buffer.extend(L[0:head])
                else:
                    self.buffer.extend(L) # in this sequence, there is no 0xA5 or 200
        else:
            # a sequence without 0xA5 or 0x5A
            bufLen = len(self.buffer)
            if bufLen == 0:
                rospy.loginfo(str(L))
            else:

                if 200 in L:
                    head = L.index(200)

                    if head == 0:
                        rospy.loginfo(str(L))
                    else:
                        self.buffer.extend(L[0:head])
                else:
                    self.buffer.extend(L)

    def synch(self, L):
        head = L.index(0xA5)
        if L[head+1] == 0xA5:
            self.x = self.hex2float(L[head+2], L[head+3])
            self.y = self.hex2float(L[head+4], L[head+5])
            self.width = (L[head+6] << 8) + L[head+7]
            self.height = (L[head+8] << 8) + L[head+9]
            self.buffer = L[head+10:]

    def hex2float(self, high, low):
        speed = low + (high << 8)
        if speed > 32768:
            speed = ~((~speed) & 0x0000FFFF)
        return float(speed) / 1000.0

    def mapping(self, LaserMap):
        rospy.loginfo("Prepare for the display using rviz")
        if 200 in LaserMap:
            H = LaserMap.index(200)
            self.map.data = LaserMap[:H]
            self.map.info.origin.position.x = self.x
            self.map.info.origin.position.y = self.y
            self.map.info.width = self.width
            self.map.info.height = self.height
            self.pub.publish(self.map)
        else:
            rospy.loginfo("this map is not valid")

    def check(self, buf):
        rospy.loginfo("self.width = " + str(self.width) + ", self.height = " + str(self.height))
        if 200 in buf:
            tail = buf.index(200)
            L = buf[:tail]
            wph = self.width * self.height
            rospy.loginfo("len(L) = " + str(len(L)) + ", width*height = " + str(wph))
            if len(L) == self.width * self.height:
                return True
            else:
                return False
        else:
            if len(buf) == self.width * self.height:
                return True
            else:
                return False

    def quatilization(self,x):
        if x == 255:
            return -1
        else:
            return x

if __name__ == '__main__':
    try:
    	dp()
    	rospy.spin()
    except:
    	pass
