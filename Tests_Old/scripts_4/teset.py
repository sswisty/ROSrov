#!/usr/bin/python
"""
# -*- coding: utf-8 -*-
#
#Testing code for using xbox controller
#
#If xboxdrv is not running the axes for the controller are off
#    The right thumb y axis is read as the right trigger, noticed this before 
#    with ROV-Siren code. Changes in that repository should be sufficient to 
#    work here. If xboxdrv is running the controller is read properly.
#    
#    Ltrigger -> Rthumb x
#    Rthumb x -> Rtrigger
#    Rthumb y -> Rthumb x
#    Rtrigger -> Ltrigger
#
#Created on Fri Apr 21 09:58:01 2017
#
#@author: sswisty
#

#import pygame
#from pygame.locals import *
#import os, sys
#import threading
#import time
"""

import XboxController2
import rospy
from std_msgs.msg import String

XboxCont = XboxController2.XboxController(
    controllerCallBack = None,
    joystickNo = 0,
    deadzone = 15,
    scale = 100,
    invertYAxis = True)
    


XboxCont.start()

def comms():
    pub = rospy.Publisher('Values', String, queue_size = 10)
    rospy.init_node('DriveStation', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        #data = "A {}, B {}, X {}, Y{} %s".format(round(XboxCont.A), round(XboxCont.B), round(XboxCont.X), round(XboxCont.Y)) % rospy.get_time()
        data = "hello world %s" % rospy.get_time()        
        rospy.loginfo(data)
        print 'Publishing...'
        pub.publish(data)
        rate.sleep()
        
if __name__ == '__main__':
    try:
        comms()
        print 'Here'
    except rospy.ROSInterruptException:
        pass
        XboxCont.stop()
    


#while Operational:
    
#    try:
       # print "RTrigger {}, LTrigger {}".format(round(XboxCont.RTRIGGER), round(XboxCont.LTRIGGER))
       # print "LThumb x {}, LThumb y {}".format(round(XboxCont.LTHUMBX), round(XboxCont.LTHUMBY))
       # print "RThumb x {}, RThumb y {}".format(round(XboxCont.RTHUMBX), round(XboxCont.RTHUMBY))
#        print "A {}, B {}, X {}, Y{}".format(round(XboxCont.A), round(XboxCont.B), round(XboxCont.X), round(XboxCont.Y))
       # print " "
       # print " "
       
#    except KeyboardInterrupt:
#        XboxCont.stop()
#        print "User Cancled"
#        Operational = False
    
#except KeyboardInterrupt:
#    print 'Cancled"
#    XboxCont.stop()