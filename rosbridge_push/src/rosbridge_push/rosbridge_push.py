#!/usr/bin/env python

import websocket
import rospy
import json
from rosbridge_library.rosbridge_protocol import RosbridgeProtocol

class RosbridgePushClient():

    def __init__(self, url):
        self.url = url
        parameters = {
            "fragment_timeout": 600,
            "delay_between_messages": 0,
            "max_message_size": None,
            "unregister_timeout": 10,
            "bson_only_mode": False
        }
        self.ws = None
        self.protocol = RosbridgeProtocol("main", parameters=parameters)
        self.protocol.outgoing = self.send

    def connect(self):
        try:
            self.ws = websocket.create_connection(self.url)
            rospy.loginfo("Connected to: %s", self.url)
        except Exception as exc:
            rospy.logerr("Unable to connect to server.  Reason: %s", str(exc))

    def isConnected(self):
        if self.ws is None:
            return False
        else:
            return self.ws.connected

    def send(self, message):
        if self.ws.connected:
            rospy.loginfo("Sending message!")
            self.ws.send(message)

    def recv(self):
        try:
            result = self.ws.recv()
            rospy.loginfo(result);
            self.protocol.incoming(result);
        except websocket.WebSocketConnectionClosedException as exc:
            rospy.logerr("Server is not responding. Error: %s", str(exc))


    def close(self):
        # TODO: should we rebuild the Rosbridge Protocol after a disconnect?
        rospy.loginfo("Shutting down rosbridge_push")
        if self.isConnected():
            rospy.loginfo("Closed websocket to: %s", self.url)
            self.ws.close()
            self.ws = None
