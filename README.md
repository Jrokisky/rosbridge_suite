WIP.
Forked Rosbride_Suite to handle firewall issues by using a trusted external server coupled with websockets as a middleman.

* Robot reaches out to server to intiate a web socket
* PCs then send rosbridge messages to the server, which are then passed to the robot.
