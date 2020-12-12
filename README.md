This was an attempt to modify Rosbride_Suite to handle firewall issues and the lack of a static, external ip on a company networl by using a trusted external server as a switchboard and websockets to communicate.

The approach would work as followed:

* On boot, the robot reaches out to the external server to intiate a web socket
* Other PCs can then send rosbridge messages to the external server
* The external server then sends those messages, via the websocket, down to the robot

This implementation was not close to complete and buggy, but I was able to successfully transmit data from a kinect to the browser with it.
