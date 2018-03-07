#!/usr/bin/env python
# license removed for brevity
import rospy
from nav_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import *
import tf

class showpath():

	def __init__(self):
		rospy.init_node('showpath', anonymous = False)
		self.pub = rospy.Publisher('trajectory', Path, queue_size = 1000)
		self.odom_pub = rospy.Publisher('odom', Odometry, queue_size = 50)
		#rospy.loginfo('showpath is initialized...')
		self.odom_broadcaster = tf.TransformBroadcaster()
		self.marker_pub = rospy.Publisher('line', Marker, queue_size = 10)
		self.marker_wide_pub = rospy.Publisher('wideline', Marker, queue_size = 10)
		self.sub = rospy.Subscriber('/info/trajectory', Pose2D, self.callback, queue_size = 10)
		self.path = Path()
		self.path.header.stamp = rospy.Time.now()
                self.path.header.frame_id = 'odom'
		self.this_pose_stamped = PoseStamped()

		self.marker = Marker()
		self.marker1 = Marker()
		self.marker.id = 0
		self.marker.ns = "basic_shapes"
		self.marker.type = 4
		self.marker.action = 0	
		self.marker1.id = 1
		self.marker1.ns = "wide_line"
		self.marker1.type = 4
		self.marker1.action = 0	

                rospy.loginfo('showpath is initialized...')
		#self.point = Point()
		#self.goal_quat = Quaternion()


	def callback(self, data):
		#rospy.loginfo("data_received "+str(data.x)+" "+str(data.y))
		self.drawpath(data)

	def drawpath(self, data):
		self.this_pose_stamped.pose.position.x = data.x
		self.this_pose_stamped.pose.position.y = data.y
        	#self.goal_quat = tf.createQuaternionMsgFromYaw(data.theta)
	        q = tf.transformations.quaternion_from_euler(0, 0, data.theta)
        	goal_quat = Quaternion(*q)
		self.q = q
		self.this_pose_stamped.pose.orientation.x = goal_quat.x
		self.this_pose_stamped.pose.orientation.y = goal_quat.y
		self.this_pose_stamped.pose.orientation.z = goal_quat.z
		self.this_pose_stamped.pose.orientation.w = goal_quat.w
		self.this_pose_stamped.header.stamp = rospy.Time.now()
		self.this_pose_stamped.header.frame_id = 'odom'
		self.path.poses.append(self.this_pose_stamped)
		#rospy.loginfo(str(len(self.path.poses)))
		self.pub.publish(self.path)
		self.move_robot()

		self.marker.header.frame_id = 'odom'
		self.marker.header.stamp = rospy.Time.now()

		point = Point()
		point.x = data.x
		point.y = data.y
		point.z = 0.0
		self.marker.frame_locked = True
		#self.marker.pose.position.z = self.point.z = 0.0
		self.marker.pose.orientation.w = 1.0
		self.marker.color.r = 0.0
		self.marker.color.g = 1.0
		self.marker.color.b = 0.0
		self.marker.color.a = 1.0
		#self.marker.colors.append(self.marker.color)
		self.marker.scale.x = 0.03
		self.marker.points.append(point)
		#self.marker.scale.y = 0.02
		#self.marker.scale.z = 0.001
		self.marker.lifetime.secs = 1000
		self.marker.lifetime.nsecs = 0.0

		#self.markers.markers.append(self.marker)
		self.marker_pub.publish(self.marker)

		self.marker1.header.frame_id = 'odom'
		self.marker1.header.stamp = rospy.Time.now()

		self.marker1.frame_locked = True
		#self.marker.pose.position.z = self.point.z = 0.0
		self.marker1.pose.orientation.w = 1.0
		self.marker1.color.r = 1.0
		self.marker1.color.g = 1.0
		self.marker1.color.b = 1.0
		self.marker1.color.a = 0.3
		#self.marker.colors.append(self.marker.color)
		self.marker1.scale.x = 0.2
		self.marker1.points.append(point)
		#self.marker.scale.y = 0.02
		#self.marker.scale.z = 0.001
		self.marker1.lifetime.secs = 100
		self.marker1.lifetime.nsecs = 0.0
		self.marker_wide_pub.publish(self.marker1)

		#self.this_pose_stamped.header.stamp
	def move_robot(self):
		self.odom_broadcaster.sendTransform((self.this_pose_stamped.pose.position.x, self.this_pose_stamped.pose.position.y, 0), self.q, self.this_pose_stamped.header.stamp, "base_link", "odom")
		odom = Odometry()
		odom.header.stamp = self.this_pose_stamped.header.stamp
		odom.header.frame_id = 'base_link'
		odom.pose.pose.position = self.this_pose_stamped.pose.position
		odom.pose.pose.position = self.this_pose_stamped.pose.orientation
		odom.child_frame_id = 'base_link'
		odom.twist.twist.linear.x = 0
		odom.twist.twist.linear.y = 0
		odom.twist.twist.angular.z = 0
		self.odom_pub.publish(odom)


if __name__ == '__main__':
	try:
		showpath()
		rospy.spin()
	except:
		pass
