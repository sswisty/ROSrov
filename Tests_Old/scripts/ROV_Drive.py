#!/usr/bin/env python
"""
ROS-ROV Drive Station Control System

This creates the Drive Station node that publishes drive commands to ****
The ROV subscribes to the ***** topic and uses the commands to manuver
The ROV publishes temperature/depth/salinity measurements to *****
A separate node can be launched to view the measurements.

TO DRIVE ROV:
    Start a roscore (?)
    In a new terminal 'source rossorce.bash'. This allows for rosrun of this package
    'rosrun ros_rov THISFILE
    In a new terminal ssh into the ROV
    'source rovsource.bash'. This allows for rosrun of ROV codes
    'rosrun rov_pkg ROV_FILE
    

Created on Sun Apr 23 14:48:17 2017

@author: sswisty
"""

import rospy
from std_msgs.msg import String
import XboxController2

XboxCont = XboxController2.XboxController(
    controllerCallBack = None,
    joystickNo = 0,
    deadzone = 15,
    scale = 100,
    invertYAxis = True)


def Comms():
    pub = rospy.Publisher('DriveCommands', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    XboxCont.start()
    while not rospy.is_shutdown():
        #UpdateButtons()
        Aval = XboxCont.A
        hello_str = 'Value of A {}'.format(Aval) # % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        Comms()
    except rospy.ROSInterruptException:
        pass