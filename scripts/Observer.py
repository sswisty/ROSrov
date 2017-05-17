#!/usr/bin/env python
"""
Created on Tue May  2 17:42:56 2017

@author: sswisty
"""


import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3


class Observer():
    
    def PressureCallback(self, PTmsg):
        # Read pressure, tempature from topic 
        self.PTreadings = PTmsg
        #rospy.loginfo(self.PTreadings)
        
    def IMUcallback(self, IMUmsg):
        # Read IMU from topic
        self.phi = IMUmsg.angular.x
        self.theta = IMUmsg.angular.y 
        self.omega = IMUmsg.angular.z
        #rospy.loginfo(self.IMUreadings)
        
    def vel_callback(self, feedback_msg):
        self.surge = feedback_msg.linear.x
        self.sway = feedback_msg.linear.y
        self.heave = feedback_msg.linear.z
        self.roll = feedback_msg.angular.x # currently set to zero
        self.pitch = feedback_msg.angular.y
        self.yaw = feedback_msg.angular.z
        
    def shutdown(self):
        """ publish an empty twist message to stop the turtlebot"""
        rospy.loginfo("Stopping ROV")
        rospy.sleep(1)
        
        
    def __init__(self):
        rospy.init_node('Observer')
        rospy.on_shutdown(self.shutdown)
        
        # Message types
        self.twist_msg = Twist()
        self.PTreadings = String()
        self.IMUreadings = String()
        self.surge = 0
        self.sway = 0
        self.heave = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.phi = 0
        self.theta = 0
        self.omega = 0
        
        # Loop Rate
        r = rospy.Rate(50)
        
        # SUBSCRIBERS
        self.cmd_vel = rospy.Subscriber("DriveCommands", Twist, self.vel_callback)
        self.PressTemp_sub = rospy.Subscriber("PT_data", String, self.PressureCallback)
        self.IMU_sub = rospy.Subscriber("IMU_data", Twist, self.IMUcallback)

        
        while not rospy.is_shutdown():
            
            print 'Surge: {}'.format(self.surge)
            print self.PTreadings
            print 'IMU DATA: Roll: {}, Pitch: {}, Yaw: {}'.format(self.phi, self.theta, self.omega)
            
            r.sleep()
            
        return
        

if __name__ == '__main__':
    try:
        Observer()        
    except rospy.ROSInterruptException:
        print 'Mission Aborted'
        pass
