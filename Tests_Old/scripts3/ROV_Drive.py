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
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import XboxController2

XboxCont = XboxController2.XboxController(
    controllerCallBack = None,
    joystickNo = 0,
    deadzone = 15,
    scale = 100,
    invertYAxis = True)


class ROVcomms():
    
    def UpdateControls(self):
        
        # Get translational motion
        surge = XboxCont.LTHUMBY
        sway = XboxCont.LTHUMBX
        if XboxCont.RTRIGGER != 0:
            heave = XboxCont.RTRIGGER
        elif XboxCont.LTRIGGER != 0:
            heave = -XboxCont.LTRIGGER
        else:
            heave = 0
            
        # Get rotational motion
        yaw = XboxCont.RTHUMBY
        pitch = XboxCont.RTHUMBX
        roll = 0
        
        # Create a Twist message to publihse
        self.twist_msg = Twist(Vector3(surge, sway, heave),Vector3(roll, pitch, yaw))
        
        # Publish the message
        self.cmd_vel.publish(self.twist_msg)
        
    def PressureCallback(self, PTmsg):
        # Read pressure, tempature from topic 
        self.PTreadings = PTmsg
        #rospy.loginfo(self.PTreadings)
        
    def IMUcallback(self, IMUmsg):
        # Read IMU from topic
        self.IMUreadings = IMUmsg
        #rospy.loginfo(self.IMUreadings)
        
    def shutdown(self):
        """ publish an empty twist message to stop the turtlebot"""
        rospy.loginfo("Stopping ROV")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
        
        
    def __init__(self):
        rospy.init_node('Admiral')
        rospy.on_shutdown(self.shutdown)
        XboxCont.start()
        
        # Message types
        self.twist_msg = Twist()
        self.PTreadings = String()
        
        # Loop Rate
        r = rospy.Rate(50)
        
        # SUBSCRIBERS AND PUBLISHERS
        # Publish drive commands for the ROV
        self.cmd_vel = rospy.Publisher("DriveCommands", Twist,queue_size=100)
        
        # Subscrive to sensor measurements
        self.PressTemp_sub = rospy.Subscriber("PT_data", String, self.PressureCallback)
        self.IMU_sub = rospy.Subscriber("IMU_data", String, self.IMUcallback)
        #self.test = rospy.Subscriber()
        
        while not rospy.is_shutdown():
            
            # Update control values, send to ROV            
            self.UpdateControls()
            
            if XboxCont.A == 1:
                print self.PTreadings
                print 'A is pressed'

            r.sleep()
        return
        

if __name__ == '__main__':
    try:
        ROVcomms()        
    except rospy.ROSInterruptException:
        print 'Mission Aborted'
        pass