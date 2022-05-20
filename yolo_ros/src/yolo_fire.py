#! /usr/bin/env python3
import rospy
from std_msgs.msg import String
from yolov5.detect import run, parse_opt

def talker(opt):
    pub = rospy.Publisher('yolodetection', String, queue_size=10)
    rospy.init_node('yolo', anonymous=True)
    while not rospy.is_shutdown():
        print('hello baby')
        results = run(**vars(opt))
        print(type(results))
        print(str(results).split(':')[1])
        print('hihihihihihihihihihihihihihihi')
        hello_str = str(results)
        pub.publish(hello_str)

if __name__ == '__main__':
    try:
        opt = parse_opt()
        talker(opt)        
    except rospy.ROSInterruptException:
        pass