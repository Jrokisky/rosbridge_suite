#!/usr/bin/env python

import websocket
import rospy
from rosbridge_library.rosbridge_protocol import RosbridgeProtocol

class RosbridgePushClient():

    def __init__(self, url):
        self.url = url
        self.protocol = RosbridgeProtocol("main")
        self.protocol.outgoing = self.send
        self.connected = False

    def connect(self):
        try:
            self.ws = websocket.create_connection(self.url)
            self.connected = True
            rospy.loginfo("Connected to: %s", self.url)
        except Exception as exc:
            rospy.logerr("Unable to connect to server.  Reason: %s", str(exc))
            self.connected = False

    def isConnected(self):
        return self.connected

    def send(self, message):
        rospy.loginfo("Sending message!")
        self.ws.send(message)

    def recv(self):
        result = self.ws.recv()
        rospy.loginfo(result);
        self.protocol.incoming(result);

    def close(self):
        rospy.loginfo("Shutting down rosbridge_push")
        if self.isConnected():
            rospy.loginfo("Closed websocket to: %s", self.url)
            self.ws.close()
            self.connected = False
