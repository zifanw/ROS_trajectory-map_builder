#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from visualization_msgs.msg import *
class dp():

	def __init__(self):
		rospy.init_node('dp', anonymous=True)
		rospy.loginfo('The infocenter is initilized...')
		self.pub1 = rospy.Publisher('/info/trajectory', Pose2D, queue_size = 100)
                self.pub3 = rospy.Publisher('/info/status', Marker, queue_size = 10000)
		rospy.wait_for_message('read',String)
		rospy.Subscriber('read',String, self.callback, queue_size=1)

                self.text_marker = Marker()
                self.text_marker.header.frame_id = 'odom'
                self.text_marker.id = 3
                self.text_marker.ns = "text"
                self.text_marker.type = 9
                self.text_marker.action = 0
                self.text_marker.frame_locked = True
                self.text_marker.color.r = 1.0
                self.text_marker.color.g = 1.0
                self.text_marker.color.b = 1.0
                self.text_marker.color.a = 1.0
                self.text_marker.scale.z = 0.5

	def callback(self,data):
		#rospy.loginfo(rospy.get_caller_id() + "/n I heard %s", data.data)
                #self.postdata = data.data
                #ending = self.postdata.index('\n')

                msg = data.data.split(',')
                self.decom(msg)

        def decom(self, data):
                if len(data) > 5 and 'AA' in data:
                        head = data.index('AA')
                        self.sendpath(data[head:head+5])
                        self.decom(data[head+5:])
                elif 'AA' in data:
                        head = data.index('AA')
                        data.append(0)
                        if len(data) > 5:
                              self.sendpath(data[head:head+5])

        def sendpath(self, data):
     		if data[0] == 'AA':
			pose = Pose2D()
                        self.text_marker.text = data[1]
     			pose.x = float(data[2])
     			pose.y = float(data[3])
			pose.theta = float(data[4])
                        self.pub1.publish(pose)
                        self.pub3.publish(self.text_marker)
                       # pose.x = float(data[5])
                       # pose.y = float(data[6])
                       # pose.theta = float(data[7])
     		#	self.pub2.publish(pose)
		else:
			rospy.loginfo("not a valid data")

if __name__ == '__main__':
    try:
    	dp()
    	rospy.spin()
    except:
    	pass
