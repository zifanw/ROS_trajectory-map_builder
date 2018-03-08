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
		rospy.wait_for_message('read',String)
		rospy.Subscriber('read',String, self.callback, queue_size=1)

	def callback(self,data):
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
     			pose.x = float(data[2])
     			pose.y = float(data[3])
			pose.theta = float(data[4])
                        self.pub1.publish(pose)
		else:
			rospy.loginfo("not a valid data")

if __name__ == '__main__':
    try:
    	dp()
    	rospy.spin()
    except:
    	pass
