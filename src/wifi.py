#!/usr/bin/env python
# license removed for brevity
import rospy
import threading
import time
from std_msgs.msg import String
import socket

class myconnection():
    def __init__(self, host, port):
        rospy.init_node('TCPIP_client', anonymous=True)
        self.pub = rospy.Publisher('read', String, queue_size = 1024)
        self.s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        #self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind((host,port))
        rospy.loginfo("The server is started...")
        self.s.listen(5)
        rospy.loginfo("Waiting for connection")
        while True:
            client,addr=self.s.accept()
            t=threading.Thread(target=self.tcplink(client,addr))

    def tcplink(self, client, addr):
        size = 1024
        print 'Accept new connection from %s:%s...' % addr
        while True:
            try:
                result = String()
                result.data =client.recv(size)
                if result.data:
                    print result.data
                    self.pub.publish(result)
                else:
                    raise error('Client disconnected')
            except:
                client.close()
                print 'Connection from %s:%s closed.'%addr
                return False

if __name__ == '__main__':
    try:
    	myconnection('192.168.3.168',60000) # port ID 60000 is set for the trajectory
    	rospy.spin()
    except:
    	pass
