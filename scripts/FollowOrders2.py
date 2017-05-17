#!/usr/bin/env python
"""
Created on Tue Apr 25 12:00:27 2017

@author: sswisty
"""



import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import random


class ROVdriver():
    
    def vel_callback(self, feedback_msg):
        self.surge = feedback_msg.linear.x
        self.sway = feedback_msg.linear.y
        self.heave = feedback_msg.linear.z
        self.roll = feedback_msg.angular.x # currently set to zero
        self.pitch = feedback_msg.angular.y
        self.yaw = feedback_msg.angular.z
        
    def TakeMeasurements(self):
        #self.Temp = random.randint(1, 100)
        
        #self.Pressure = random.randint(1, 100)
        
        
        if self.cycles > 5:
            self.Temp += .1
            self.Pressure +=.1
        if self.cycles > 10:
            self.Temp -= .1
            self.Pressure -= .1
        if self.cycles > 20:
            self.cycles = 0
        
        self.PTdata = 'Temp: {} (C), Pressure: {} (ft)'.format(self.Temp, self.Pressure)
        
        self.PressTemp_pub.publish(self.PTdata)
        
        # Similar for IMU data
        
    def IMUsend(self):
        phi = 2.5
        theta = -5.6
        omega = 92.4
        self.twist_msg = Twist(Vector3(0, 0, 0),Vector3(phi, theta, omega))
        
        # Publish the message
        self.IMU_pub.publish(self.twist_msg)
        
    def shutdown(self):
        """ publish an empty twist message to stop the turtlebot"""
        rospy.loginfo("Stopping ROV")
        rospy.sleep(1)
    
    def __init__(self):
        rospy.init_node('ROV_Pilot')
        rospy.on_shutdown(self.shutdown)

        
        # Message types
        self.twist_msg = Twist()
        self.PTdata = String()
        self.surge = 0
        self.sway = 0
        self.heave = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.cycles = 0
        self.Temp = 17.2
        self.Pressure = 1.3
        
        
        # Loop Rate
        r = rospy.Rate(50)
        
        # SUBSCRIBERS AND PUBLISHERS
        # Subscribe to drive commands for the ROV
        self.cmd_vel = rospy.Subscriber("DriveCommands", Twist,self.vel_callback)
        
        # Publish sensor measurements
        self.PressTemp_pub = rospy.Publisher("PT_data", String, queue_size=100)
        self.IMU_pub = rospy.Publisher("IMU_data", Twist, queue_size=100)
        #self.test = rospy.Subscriber()
        
        while not rospy.is_shutdown():
            
            self.TakeMeasurements()
            self.IMUsend()
            
            print self.surge, self.sway, self.heave, self.pitch, self.yaw
            
            self.cycles += 1
            
           # print self.cycles
            
            
            r.sleep()
        return
        

if __name__ == '__main__':
    try:
        ROVdriver()        
    except rospy.ROSInterruptException:
        print 'Mission Aborted'
        pass
