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
        #rospy.loginfo('The mapping is initilized...')
        self.pub = rospy.Publisher('map', OccupancyGrid, queue_size = 1000)
        #rospy.loginfo('The mapping is initilizing...')
        self.sync = 0
        self.map = OccupancyGrid()
        self.map.header.seq = 0
        self.map.header.frame_id = 'odom'
        self.map.info.resolution = 0.05
        self.map.info.width = 600
        self.map.info.height = 600
        self.map.info.origin.position.x = 0
        self.map.info.origin.position.y = 0
        self.map.info.origin.position.z = 0
        self.map.info.origin.orientation.x = 0
        self.map.info.origin.orientation.y = 0
        self.map.info.origin.orientation.z = 0
        self.map.info.origin.orientation.w = 1
        self.buffer = []
        rospy.Subscriber('/read/map',String, self.callback, queue_size=1000)
        rospy.loginfo('The mapping is initialized and waiting for transmission...')
        rospy.wait_for_message('/read/map',String)

    def callback(self, data):
        L = list(data.data)
        L = map(ord, L)
        #rospy.loginfo(L)
        if self.sync == 0 and 77 in L: # M = 77
            self.synch(L)
            rospy.loginfo("sync is successful.")
        elif self.sync:
            if len(self.buffer) < 360003 and 77 not in L:
                self.buffer.extend(L) # insert into the buffer
            elif len(self.buffer) < 360003 and 77 in L:
                head = L.index(77)
                self.buffer.extend(L[:head])
                if len(self.buffer) == 360003:
                    self.mapping(self.buffer) # create a map
                    rospy.loginfo("a complete map is found")
                    self.buffer = L[head:]
                else: #damaged map
                    self.buffer = L[head:]
                    rospy.loginfo("an incomplete map is found")
        #elif 0 in L:
        #    rospy.loginfo("found 0")
        #elif 255 in L:
        #    rospy.loginfo("found 255")
    def synch(self, L):
        head = L.index(77)
        if L[head+1] == 65 and L[head+2] == 80: # A = 65 P = 80
            self.sync = 1
            self.buffer = L[head:]

    def mapping(self, HitMap):
        if len(HitMap) == 360003 and HitMap[0] == 77:
            self.map.data = map(self.quatilization, HitMap[3:])
            self.map.header.stamp = rospy.Time.now()
            self.map.info.map_load_time = self.map.header.stamp
            self.pub.publish(self.map)

    def quatilization(self, x):
        if x == 127:
            return -1 # unknown
        elif x == 255 :
            return 50 # path
        elif x == 0: # barrier
            return 100
        else:
            return -1


if __name__ == '__main__':
    try:
    	dp()
    	rospy.spin()
    except:
    	pass
