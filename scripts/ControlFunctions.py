# ================================================================
#      ----------     Necessary libraries   ------------
from Adafruit_PWM_Servo_Driver import PWM
# from ControlFunctions import MotorControl, WriteMotor, Correction

#import socket
#import sys

#from SendReceive import SendReceive

#import time     # may not be necessary for final build

# ===================================================================
#    ----------------- Inatilization commands ----------------

# Define the hat over the I2C connection pins
hat = PWM(0x40)

# Set the desired frequency for the servos (50 Hz)
f = 48
hat.setPWMFreq(f)

# Define the thruster pins on the ServoHat
global thruster1
global thruster2
global thruster3
global thruster4
global thruster5
global thruster6

thruster1 = 0;
thruster2 = 2;
thruster3 = 4;
thruster4 = 6;
thruster5 = 8;
thruster6 = 10;
thruster7 = 12;
thruster8 = 14;


center = 307    # Use this to initilize the thrusters


# =====================================================================
# =====================================================================
#               CONTROL FUNCTIONS
# =====================================================================
# =====================================================================

def MotorControl(RX,RY,LX,LY,heave):

    # ================ XY Plane Motion =========================

    if (RX == 0):  # No Yaw control
        if (LY != 0 and LX == 0):
            # Only Forward/Backward
            WriteMotor(thruster1, LY)
            WriteMotor(thruster2, LY)
            WriteMotor(thruster3, LY)
            WriteMotor(thruster4, LY)
        elif (LY == 0 and LX != 0):
            # Only Left/Right
            WriteMotor(thruster1, LX)
            WriteMotor(thruster2, -LX)
            WriteMotor(thruster3, LX)
            WriteMotor(thruster4, -LX)
        else:
            # X and Y plane motion
            WriteMotor(thruster1, Correction(LY, LX, '+'))
            WriteMotor(thruster2, Correction(LY, LX, '-'))
            WriteMotor(thruster3, Correction(LY, LX, '+'))
            WriteMotor(thruster4, Correction(LY, LX, '-'))
            
    elif (RX != 0):   # Yaw Control
        if (LY == 0 and LX == 0):
            # Only Yaw (CW and CCW)
            WriteMotor(thruster1, RX)
            WriteMotor(thruster2, -RX)
            WriteMotor(thruster3, -RX)
            WriteMotor(thruster4, RX)
        elif (LY != 0 and LX == 0):
            # Yaw and Forward/Backward
            WriteMotor(thruster1, Correction(LY, RX, '+'))
            WriteMotor(thruster2, Correction(LY, RX, '-'))
            WriteMotor(thruster3, Correction(LY, RX, '-'))
            WriteMotor(thruster4, Correction(LY, RX, '+'))
        elif (LY == 0 and LX != 0):
            # Yaw and Left/right
            WriteMotor(thruster1, Correction(LX, RX, '+'))     # Yaw and left/right
            WriteMotor(thruster2, Correction(-LX, -RX, '-'))
            WriteMotor(thruster3, Correction(LX, -RX, '+'))
            WriteMotor(thruster4, Correction(-LX, RX, '-'))
        elif (LY != 0 and LX != 0):
            # Yaw and general XY Plane motion...
            #NOT CURRENTLY DOING THIS! Instead general xy-motion
            WriteMotor(thruster1, Correction(LY, LX, '+'))
            WriteMotor(thruster2, Correction(LY, LX, '-'))
            WriteMotor(thruster3, Correction(LY, LX, '+'))
            WriteMotor(thruster4, Correction(LY, LX, '-'))

    # ================ Vertical Motion =========================

    if (RY == 0):
        # No Pitch control
         if (heave != 0):
            # Descend
            WriteMotor(thruster5, heave)   
            WriteMotor(thruster6, heave)   
            WriteMotor(thruster7, heave)   
            WriteMotor(thruster8, heave)


    elif (RY != 0):
        # Pitch control
        print '!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'
        print 'PITCH SHOULD HAPPEN NOW!'
        print '!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'
        if (heave <= 0):
            # Descend + Pitch
            WriteMotor(thruster5, Correction(heave, -RY, '-'))  # Pitch and down
            WriteMotor(thruster6, Correction(heave, -RY, '-'))
            WriteMotor(thruster7, Correction(heave, RY, '-'))
            WriteMotor(thruster8, Correction(heave, RY, '-'))
        elif (heave >= 0):  # Pitch and up
            # Ascend + Pitch
            WriteMotor(thruster5, Correction(heave, -RY, '+'))
            WriteMotor(thruster6, Correction(heave, -RY, '+'))
            WriteMotor(thruster7, Correction(heave, RY, '+'))
            WriteMotor(thruster8, Correction(heave, RY, '+'))



    # This control code includes no roll control. That will be added later


def WriteMotor(Thruster, Value):
    # This writes the PWM setting to the ESC
    hat.setPWM(Thruster, 0, center + Value)
    # Value should be limited to +- 40 to keep under power restrictions



def Correction(value1, value2, sign):
    # For multi-axis motion we need to determine how much thrust each thruster
    # provides. One value is more important and thus provides a larger portion
    
    if (sign == '+'):
        correctvalue = value1/2 + (value1*value2)/80
    elif (sign == '-'):
        correctvalue = value1/2 - (value1*value2)/80

    return correctvalue    
