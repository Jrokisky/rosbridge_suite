#!/usr/bin/env python

from __future__ import print_function
import websocket
import rospy

class RosbridgePushClient():

    def __init__(self, url):
        self.url = url

    def connect(self):
        self.ws = websocket.create_connection(self.url)

    def send(self, message):
        rospy.logdebug(message)
        self.ws.send(message)

    def recv(self):
        result = self.ws.recv()
        print("'%s'" % result)

    def close(self):
        self.ws.close()
