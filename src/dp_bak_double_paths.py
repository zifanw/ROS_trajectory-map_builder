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
            self.pub2 = rospy.Publisher('/info/odom', Pose2D, queue_size = 100)
	    rospy.wait_for_message('read',String)
	    rospy.Subscriber('read',String, self.callback, queue_size=1000)
            self.marker_text_pub = rospy.Publisher('text', Marker, queue_size = 10000)
            self.text_marker = Marker()
            self.pose = Pose2D()
            self.pose.x = -2.0
            self.pose_odom = Pose2D()
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
	def callback(self,data):

            postdata = data.data[:-3]
            self.postdata = postdata.split(',')
         
            if self.postdata[0] == 'AA':
                rospy.loginfo("received data")
                self.pose.x = float(self.postdata[2])-2.0
                self.pose.y = float(self.postdata[3])
                self.pose.theta = float(self.postdata[4])
            if len(self.postdata) > 7:
                self.pose_odom.x = float(self.postdata[5])
                self.pose_odom.y = float(self.postdata[6])
                self.pose_odom.theta = float(self.postdata[7])	
                self.pub2.publish(self.pose_odom)
                self.pub1.publish(self.pose)

            self.text_marker.header.stamp = rospy.Time.now()
		#self.marker.colors.append(self.marker.color)
            self.text_marker.scale.z = 0.3
            odom_str = "Odometry: x= " + str(self.pose_odom.x)+" y=" + str(self.pose_odom.y)+ " yaw= " + str(self.pose_odom.theta)
            self.text_marker.text = "V-SLAM: x=" + str(self.pose.x) + "  y=" + str(self.pose.y) + "  yaw=" + str(self.pose.theta) + "\n" + odom_str

            self.text_marker.lifetime.secs = 10000
            self.text_marker.lifetime.nsecs = 0.0
            self.marker_text_pub.publish(self.text_marker)
           # else:
            #    rospy.loginfo("not a valid data")

if __name__ == '__main__':
    try:
    	dp()
    	rospy.spin()
    except:
    	pass
