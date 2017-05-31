# mobile_robot
The encoder.ino is the arduino file to publish the rpm and subscribe the cmd_vel and convert to corresponding rpm. 
The base_controller.cpp is the node subscribing to rpm and gyro and publishing tf and odom.
The gyro.py node binds the gyro reading and publish data under gyro topic.
These are the nodes necessary to perform navigation using the navigation stack of ROS 
