#!/usr/bin/python
# -*- coding: utf-8 -*-
import socket
import threading
import time

def tcplink(sock,addr):
    print 'Accept new connection from %s:%s...' % addr
    sock.send('Welcome!')
    while True:
        data=sock.recv(1024)
        print data
        time.sleep(1)
        if data=='exit' or not data:
            break
        sock.send('Hello,%s!'%data)
    sock.close()
    print 'Connection from %s:%s closed.'%addr

if __name__ == '__main__':
    s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)

    s.bind(('192.168.3.142',60000))
    s.listen(5)
    print 'Waiting for connection...'
    while True:

    #接受一个新连接
        sock,addr=s.accept()

    #创建新线程来处理TCP连接
        t=threading.Thread(target=tcplink(sock,addr))
