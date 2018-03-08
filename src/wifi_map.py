#!/usr/bin/env python
# license removed for brevity
import rospy
import threading
import time
from std_msgs.msg import String
import socket

class myconnection():
    def __init__(self, host, port):
        rospy.init_node('TCPIP_map_server', anonymous=True)
        self.pub = rospy.Publisher('/read/map', String, queue_size = 1024)
        self.s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        #self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind((host,port))
        rospy.loginfo("The server is started...")
        self.s.listen(5)
        rospy.loginfo("Waiting for connection")
        while True:
            client,addr=self.s.accept()
            #client.settimeout(60)
            t=threading.Thread(target=self.tcplink(client,addr))

    def tcplink(self, client, addr):
        size = 1024
        result = String()
        print 'Accept new connection from %s:%s...' % addr
        while True:
            try:
                data = client.recv(size)
                if data:
                    print len(data)
                    result.data = data
                    self.pub.publish(result)
                else:
                    raise error('Client disconnected')
            except:
                    client.close()
                    print 'Connection from %s:%s closed.'%addr
                    return False # re-connection

if __name__ == '__main__':
    try:
    	myconnection('192.168.3.168',60000) # port ID 60001 is set for the map
    	rospy.spin()
    except:
    	pass
