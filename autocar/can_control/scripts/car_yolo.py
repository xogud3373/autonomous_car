#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Int64
from can_module import CAN
from pygame import time
from can_msg.msg import control

PARKING = 0
DRIVE = 5
NEUTRAL = 6
REVERSE = 7
FPS = 50


class CAN_YOLO:
    def __init__(self):
        rospy.init_node('can_yolo', anonymous=True)
        rospy.Subscriber('yolodetection', Int64, self.clbk_yolo)
        self.pub = rospy.Publisher('controller', control, queue_size=1)
        self.control_msg = control()
        self.acc_num = 0
        self.acc_num1 = 0

    def clbk_yolo(self, msg):
        print(self.acc_num1)
        if msg.data == 0 and self.acc_num1 > 10:
           self.control_msg.accel_x = 0
           self.control_msg.theta = 0
           #self.control_msg.gear = 5
           self.control_msg.override = True
           self.pub.publish(self.control_msg)
           print("FIND PERSON")

        elif self.acc_num == 0:
            self.control_msg.accel_x = 1
            self.control_msg.theta = 0
            #self.control_msg.gear = 5
            self.control_msg.override = True
            self.pub.publish(self.control_msg) 
            print("GOGOGOGOGOGOGO")
            self.acc_num = self.acc_num + 1
        self.acc_num1 = self.acc_num1 + 1 


if __name__ == '__main__':
    can_cmd = CAN_YOLO() 
    rospy.spin()
