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
from Adafruit_PWM_Servo_Driver import PWM
from ControlFunctions import MotorControl
import smbus
import time




class ROVdriver():
    
    def vel_callback(self, feedback_msg):
        self.surge = feedback_msg.linear.x
        self.sway = feedback_msg.linear.y
        self.heave = feedback_msg.linear.z
        self.roll = feedback_msg.angular.x # currently set to zero
        self.pitch = feedback_msg.angular.y
        self.yaw = feedback_msg.angular.z
        
    def TakeMeasurements(self):
        self.Temp = random.randint(1, 100)
        self.Pressure = random.randint(1, 100)
        
        # Read 12 bytes of calibration data
    # Read pressure sensitivity
        data = self.bus.read_i2c_block_data(0x76, 0xA2, 2)
        C1 = data[0] * 256 + data[1]

    # Read pressure offset
        data = self.bus.read_i2c_block_data(0x76, 0xA4, 2)
        C2 = data[0] * 256 + data[1]

    # Read temperature coefficient of pressure sensitivity
        data = self.bus.read_i2c_block_data(0x76, 0xA6, 2)
        C3 = data[0] * 256 + data[1]

    # Read temperature coefficient of pressure offset
        data = self.bus.read_i2c_block_data(0x76, 0xA8, 2)
        C4 = data[0] * 256 + data[1]

    # Read reference temperature
        data = self.bus.read_i2c_block_data(0x76, 0xAA, 2)
        C5 = data[0] * 256 + data[1]

    # Read temperature coefficient of the temperature
        data = self.bus.read_i2c_block_data(0x76, 0xAC, 2)
        C6 = data[0] * 256 + data[1]

    # MS5837_30BA01 address, 0x76(118)
    #		0x40(64)	Pressure conversion(OSR = 256) command
        self.bus.write_byte(0x76, 0x40)

        time.sleep(0.25)

    # Read digital pressure value
    # Read data back from 0x00(0), 3 bytes
    # D1 MSB2, D1 MSB1, D1 LSB
        value = self.bus.read_i2c_block_data(0x76, 0x00, 3)
        D1 = value[0] * 65536 + value[1] * 256 + value[2]

    # MS5837_30BA01 address, 0x76(118)
    #		0x50(64)	Temperature conversion(OSR = 256) command
        self.bus.write_byte(0x76, 0x50)

        time.sleep(0.25)

    # Read digital temperature value
    # Read data back from 0x00(0), 3 bytes
    # D2 MSB2, D2 MSB1, D2 LSB
        value = self.bus.read_i2c_block_data(0x76, 0x00, 3)
        D2 = value[0] * 65536 + value[1] * 256 + value[2]

        dT = D2 - C5 * 256
        TEMP = 2000 + dT * C6 / 8388608
        OFF = C2 * 65536 + (C4 * dT) / 128
        SENS = C1 * 32768 + (C3 * dT ) / 256
        T2 = 0
        OFF2 = 0
        SENS2 = 0

        if TEMP >= 2000 :
                T2 = 2 * (dT * dT) / 137438953472
                OFF2 = ((TEMP - 2000) * (TEMP - 2000)) / 16
                SENS2 = 0
        elif TEMP < 2000 :
                T2 = 3 *(dT * dT) / 8589934592
                OFF2 = 3 * ((TEMP - 2000) * (TEMP - 2000)) / 2
                SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 8
                if TEMP < -1500 :
                        OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500))
                        SENS2 = SENS2 + 4 * ((TEMP + 1500) * (TEMP + 1500))

        TEMP = TEMP - T2
        OFF2 = OFF - OFF2
        SENS2 = SENS - SENS2
        self.pressure = ((((D1 * SENS2) / 2097152) - OFF2) / 8192) / 10.0
        self.cTemp = TEMP / 100.0
        self.fTemp = self.cTemp * 1.8 + 32

    #return pressure, cTemp
        
        self.PTdata = 'Temp: {}, Pressure: {}'.format(self.cTemp, self.pressure)
        
        self.PressTemp_pub.publish(self.PTdata)
        
        # Similar for IMU data
        
    def shutdown(self):
        """ publish an empty twist message to stop the turtlebot"""
        rospy.loginfo("Stopping ROV")
        rospy.sleep(1)
    
    def __init__(self):
        rospy.init_node('ROV_Pilot')
        rospy.on_shutdown(self.shutdown)

        self.FirstTime = True
        
        # Message types
        self.twist_msg = Twist()
        self.PTdata = String()
        self.surge = 0
        self.sway = 0
        self.heave = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        
        # =======================================================
        # Initilization
        # Define the hat and the pressure sensor over the I2C connection pins
        self.hat = PWM(0x40)
        self.bus = smbus.SMBus(1)

        # Set the desired frequency for the servos (50 Hz)
        self.f = 48
        self.hat.setPWMFreq(self.f)

        # Define the thruster pins on the ServoHat
        self.thruster1 = 0
        self.thruster2 = 2
        self.thruster3 = 4
        self.thruster4 = 6
        self.thruster5 = 8
        self.thruster6 = 10
        self.thruster7 = 12
        self.thruster8 = 14


        self.center = 307    # Use this to initilize the thrusters

    
        # MS5837_30BA01 address, 0x76(118)
        #		0x1E(30)	Reset command
        self.bus.write_byte(0x76, 0x1E)

        
        
        # Loop Rate
        r = rospy.Rate(50)
        
        # SUBSCRIBERS AND PUBLISHERS
        # Subscribe to drive commands for the ROV
        self.cmd_vel = rospy.Subscriber("DriveCommands", Twist,self.vel_callback)
        
        # Publish sensor measurements
        self.PressTemp_pub = rospy.Publisher("PT_data", String, queue_size=100)
        self.IMU_pub = rospy.Publisher("IMU_data", String, queue_size=100)
        #self.test = rospy.Subscriber()
        
        while not rospy.is_shutdown():
            
            if self.FirstTime:
                self.hat.setPWM(self.thruster1, 0, self.center)
                self.hat.setPWM(self.thruster2, 0, self.center)
                self.hat.setPWM(self.thruster3, 0, self.center)
                self.hat.setPWM(self.thruster4, 0, self.center)
                self.hat.setPWM(self.thruster5, 0, self.center)
                self.hat.setPWM(self.thruster6, 0, self.center)
                self.hat.setPWM(self.thruster7, 0, self.center)
                self.hat.setPWM(self.thruster8, 0, self.center)
                self.FirstTime = False
            
            self.TakeMeasurements()
            
            MotorControl(self.yaw, self.pitch, self.surge, self.sway, self.heave)
            
            
            print self.surge
            
            
            
            r.sleep()
        return
        

if __name__ == '__main__':
    try:
        ROVdriver()        
    except rospy.ROSInterruptException:
        print 'Mission Aborted'
        pass
