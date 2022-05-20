#!/usr/bin/env python3
import rospy
from can_msg.msg import control, Gear
from can_module import CAN

def publish():
    rospy.init_node('test', anonymous=True)
    pub = rospy.Publisher('controller', control, queue_size=1)
    control_msg = control()
    rate = rospy.Rate(50)
    control_msg.accel_x = 1
    while not rospy.is_shutdown():
        control_msg.accel_x = control_msg.accel_x + 0.5
        control_msg.theta = 0
        control_msg.override = True
        print(control_msg.accel_x)
        pub.publish(control_msg)

        if control_msg.accel_x >= 900:
            control_msg.accel_x = -1
            control_msg.theta = 0
            control_msg.override = True
            pub.publish(control_msg)
            break
        rate.sleep()

if __name__ == '__main__':
    try:
        publish()
    except rospy.ROSInterruptException:
        pass