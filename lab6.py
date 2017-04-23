#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Apr  5 14:50:16 2017

@author: team3
"""

import rospy
import numpy as np
import tf
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from ar_track_alvar_msgs.msg import AlvarMarker
from ar_track_alvar_msgs.msg import AlvarMarkers


class Tag0():
    x = 1.707
    y = 6.395
    z = 0.631
    w = 0.775    
class Tag1():
    x = -0.202
    y = 2.116
    z = 0.993
    w = 0.117
    
class Tag2():
    x = -0.269 
    y = -1.602
    z = -0.824
    w = 0.566
    
class Tag3():
    x = 2.074
    y = 3.542
    z = -0.034
    w = 0.999
    
class Lab6Solution():
        
    def feedback_callback(self, feedback_msg):
        """ Callback to handle feedback messages so that we know the current 
            position of the turtlebot."""        
        # Store the position and orientation
        self.current_pos.pose.position.x = feedback_msg.pose.position.x
        self.current_pos.pose.position.y = feedback_msg.pose.position.y
        self.current_pos.pose.orientation.z = feedback_msg.pose.orientation.z
        self.current_pos.pose.orientation.w = feedback_msg.pose.orientation.w    
    
    def AR_marker_callback(self, feedback_msg):
        """ Callback to handle ar tag messages to determine the position of 
            the turtlebot with respect to the ar tag."""
        if len(feedback_msg.markers) >0:
            self.markers.markers.append(feedback_msg.markers[-1])

            # Initialize the turtlebots location
            if self.first_tag:
                print 'I see tag # {}'.format(feedback_msg.markers[-1].id)
                self.origin.header.seq=1
                self.origin.header.stamp.secs = rospy.get_rostime().to_sec()
                #new.header.stamp.nsecs = rospy.get_rostime().to_nsec()
                self.origin.header.frame_id= 'map'
                self.origin.pose.pose.position.z = 0.0
                self.origin.pose.pose.orientation.x = 0.0
                self.origin.pose.pose.orientation.y = 0.0
                if self.markers.markers[-1].id == 0:
                    self.origin.pose.pose.position.x= Tag0.x - self.markers.markers[-1].pose.pose.position.x
                    self.origin.pose.pose.position.y= Tag0.y - self.markers.markers[-1].pose.pose.position.z
                    self.origin.pose.pose.orientation.z = Tag0.z
                    self.origin.pose.pose.orientation.w = Tag0.w
                    
                elif self.markers.markers[-1].id == 1:
                    self.origin.pose.pose.position.x= Tag1.x + self.markers.markers[-1].pose.pose.position.z
                    self.origin.pose.pose.position.y= Tag1.y - self.markers.markers[-1].pose.pose.position.x
                    self.origin.pose.pose.orientation.z = Tag1.z
                    self.origin.pose.pose.orientation.w = Tag1.w
                    
                elif self.markers.markers[-1].id == 2:
                    self.origin.pose.pose.position.x= Tag2.x + self.markers.markers[-1].pose.pose.position.x
                    self.origin.pose.pose.position.y= Tag2.y + self.markers.markers[-1].pose.pose.position.z
                    self.origin.pose.pose.orientation.z = Tag2.z
                    self.origin.pose.pose.orientation.w = Tag2.w
                    
                elif self.markers.markers[-1].id == 3:
                    self.origin.pose.pose.position.x= Tag3.x - self.markers.markers[-1].pose.pose.position.z
                    self.origin.pose.pose.position.y= Tag3.y + self.markers.markers[-1].pose.pose.position.x
                    self.origin.pose.pose.orientation.z = Tag3.z
                    self.origin.pose.pose.orientation.w = Tag3.w
                    
                self.first_tag=False
                self.cmd_origin.publish(self.origin)
                print 'Defined origin'
#                self.origin.theta=self.marker.theta
                
    def newPose(self, WPT='0'):
        """Takes in the new waypoint and sets the new goal position"""
        new = PoseStamped()
        new.header.seq=1
        new.header.stamp.secs = rospy.get_rostime().to_sec()
        #new.header.stamp.nsecs = rospy.get_rostime().to_nsec()
        new.header.frame_id= 'map'
        
        new.pose.position.z = 0.0
        new.pose.orientation.x = 0.0
        new.pose.orientation.y = 0.0
        # Positions in the map
        if WPT == '0': # By the window
            new.pose.position.x = Tag0.x
            new.pose.position.y = Tag0.y-0.5
            new.pose.orientation.z = Tag0.z
            new.pose.orientation.w = Tag0.w
            
        elif WPT == '1': # In the middle of the hallway by the stairs
            new.pose.position.x = Tag1.x+0.5
            new.pose.position.y = Tag1.y
            new.pose.orientation.z = Tag1.z
            new.pose.orientation.w = Tag1.w
            
        elif WPT == '2': # By the bathroom
            new.pose.position.x = Tag2.x
            new.pose.position.y = Tag2.y+0.5
            new.pose.orientation.z = Tag2.z
            new.pose.orientation.w = Tag2.w
            
        elif WPT == '3': # In the hallway after the bathroom
            new.pose.position.x = Tag3.x-0.5
            new.pose.position.y = Tag3.y
            new.pose.orientation.z = -Tag3.z
            new.pose.orientation.w = Tag3.w
            
#        elif WPT == 'S': # Stop at current position
#            new.pose.position.x = self.hold_pos.pose.position.x
#            new.pose.position.y = self.hold_pos.pose.position.y
#            new.pose.orientation.z = self.hold_pos.pose.orientation.z
#            new.pose.orientation.w = self.hold_pos.pose.orientation.w
            
        # Write a new Pose message
        self.Pose = new
        
    def new_rotation(self, orientation):
        euler = tf.transformations(orientation.x, orientation.y, orientation.z, orientation.w)
        yaw = euler[1]
        quaternion = tf.transformations.quaternion_from_euler(0,0,yaw)
        return quaternion[2], quaternion[3]
        
    def __init__(self):
        """ STUFF YOU MAY WANT TO USE"""
        rospy.init_node('lab6')
        #rospy.on_shutdown(self.shutdown())
#        self.make_map()
        # loop rate
        WPT = ''
        r = rospy.Rate(50)
        self.first_tag =True
        self.origin = PoseWithCovarianceStamped()
        self.current_pos = PoseStamped()
        self.Pose = PoseStamped()
        self.markers = AlvarMarkers()
        
        # publisher for twist messages
        self.cmd_origin = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=100)
        self.cmd_pose = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=100)
        self.feedback_sub = rospy.Subscriber("/move_base/feedback", PoseStamped, self.feedback_callback)
        self.alvar_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.AR_marker_callback)        
        cntr =0
        # IN THIS LOOP ROS SENDS AND RECEIVES  
        while not rospy.is_shutdown():
#             wait until you get the first tag
            if not self.first_tag:
                self.newPose(WPT.upper())
                self.cmd_pose.publish(self.Pose)
            r.sleep()
            cntr +=1
#             Make sure that the goal is published
            if cntr>50 and not self.first_tag:
                WPT = raw_input('Pick a waypoint (AR Tag 0,1,2,3) or E to exit: ')
                
                if WPT == 'E':
                    break
#            i = 1
        return

l = Lab6Solution()
