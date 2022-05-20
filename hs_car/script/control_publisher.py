#!/usr/bin/env python3
import rospy

import numpy as np
from math import sqrt
from math import radians
from collections import deque
#import actionlib

#roslib.load_manifest('can_cmd')
#import roslib


from geopy import distance
from attrdict import AttrDict

from PIDController import PIDLongitudinalController
#from PIDController_ import LateralController
#from Client_ import PathPlanner

from novatel_oem7_msgs.msg import BESTPOS, CORRIMU, BESTVEL, HEADING2
from can_msg.msg import control


conf = AttrDict({
    'subscribers':[
        {'topic':'/novatel/oem7/bestpos', 'type': BESTPOS, 'name': 'position'},
        {'topic':'/novatel/oem7/corrimu', 'type': CORRIMU, 'name': 'imu'},
        {'topic':'/novatel/oem7/bestvel', 'type': BESTVEL, 'name': 'velocity'},
        {'topic':'/novatel/oem7/heading2', 'type': HEADING2, 'name': 'yaw'}
    ],
    'publishers':[
        {'topic':'/controller', 'type': control, 'name': 'control'}
    ]
})

# change parameters here
param = AttrDict({'max_speed': 8, 'max_accel': 1, 'destination': (126.840655,35.230453),
                  'longitudinal':{'p':1.0, 'i': 0.5, 'd':0.2},
                  'dt': 0.02,
                  'lateral':{'K':0.5}
                  })

lat = 0
lon = 0 
acc_x = 0
cur_vel = 0 
yaw = 0

NEUTRAL = 0
PARKING = 7
DRIVE = 5

class Controller:
    def __init__(self, conf=None, param=None):
        rospy.init_node('controller', anonymous=True)

        self.callbacks = {'/novatel/oem7/bestpos': self.pos_callback,
                          '/novatel/oem7/corrimu': self.imu_callback,
                          '/novatel/oem7/bestvel': self.spd_callback,
                          '/novatel/oem7/heading2': self.yaw_callback}

        self.subscribers = [rospy.Subscriber(e.topic, e.type, self.callbacks[e.topic]) for e in conf.subscribers]

        self.publishers = {e.name: rospy.Publisher(e.topic, e.type, queue_size=1) for e in conf.publishers}

        self.imu_rate = 100
        self.pub_rate = 50
        self.param = param

        self.lon_controller = PIDLongitudinalController(param)

        self.path = []
        self.init_path()
    
 



    def init_path(self):
        self.path = [(35.22994788,126.842817,4.242640687,5),
        (35.22999963,126.842673,7.071067812,5),
        (35.23033928,126.841728,8,5),
        (35.23053336,126.841188,4.898979486,5),
        (35.230556,126.841125,3,5)]
        # target = (, 126.842448822, 0, 5)
        # self.path.append(target)
        # target = (35.2303007467, 126.841890354, 0, 5)
        # self.path.append(target)

    def pos_callback(self, data):
        global lat, lon
        lat = data.lat
        lon = data.lon

    def imu_callback(self, data):
        global acc_x
        acc_x = data.longitudinal_acc * self.imu_rate

    def spd_callback(self, data):
        global cur_vel
        cur_vel = data.hor_speed 
    
    def yaw_callback(self, data):
        # 값 개튐
        global yaw
        yaw = data.heading

    def publish_controls(self, msg):
        self.publishers['control'].publish(msg)

    # get goal from ActionClient then publish Control message to Vehicle.py['/can_cmd/vehicle']
    def run_step(self):
        index = 0
        finish = len(self.path)
        rate = rospy.Rate(50)
        print('debug while loop', index, finish)
        while not rospy.is_shutdown():
            # print(f"lat: {lat}   lon: {lon}")
            tar_lat = self.path[index][0]
            tar_lon = self.path[index][1]
            if distance.distance((lat, lon), (tar_lat, tar_lon)) < 0.005:
                index += 1
                print("nextnextnextnextnextnextnextnext")
                if index >= finish:
                    break

            ct = control()
            ct.gear = self.path[index][3]
            cmd_vel = self.path[index][2]
            ct.override = True
            ct.accel_x = self.lon_controller.run_step(cur_vel, acc_x, cmd_vel)
            # print(ct.accel_x)
            # ct.accel_x = 150 # lat_controller 써서 구하면 되는거
            ct.theta = 0 
            self.publish_controls(ct)
            #print("control is published")
            rate.sleep()
        print("finish")
        finish = control()
        finish.gear = 5
        finish.accel_x = -1
        finish.theta = 0
        finish.override = True

        self.publish_controls(finish)


    # send stop signal to vehicle
    def emergency_stop(self):
        pass


if __name__ == '__main__':
    cont = Controller(conf, param)
    cont.run_step()

