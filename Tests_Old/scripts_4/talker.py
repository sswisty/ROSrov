#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import XboxController2

XboxCont = XboxController2.XboxController(
    controllerCallBack = None,
    joystickNo = 0,
    deadzone = 15,
    scale = 100,
    invertYAxis = True)



def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    XboxCont.start()
    while not rospy.is_shutdown():
        Aval = XboxCont.A
        hello_str = 'Value of A {}'.format(Aval) # % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass