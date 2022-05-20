#!/usr/bin/env python3

import rospy

import numpy as np
from math import sqrt
from math import radians
from collections import deque
import actionlib

#roslib.load_manifest('can_cmd')
#import roslib


from geopy import distance
from attrdict import AttrDict

#from PIDController_ import PIDLongitudinalController
#from PIDController_ import LateralController
from Client_ import PathPlanner

from novatel_oem7_msgs.msg import BESTPOS, CORRIMU, BESTVEL, HEADING2
from can_msg.msg import MoveVehicleAction, MoveVehicleResult
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

global lat, lon, acc_x, cur_vel, yaw

NEUTRAL = 6
PARKING = 0
DRIVE = 5
REVERSE = 7
FPS = 50

class Controller:
    def __init__(self, conf=None, param=None):

        self.callbacks = {'/novatel/oem7/bestpos': self.pos_callback,
                          '/novatel/oem7/corrimu': self.imu_callback,
                          '/novatel/oem7/bestvel': self.spd_callback,
                          '/novatel/oem7/heading2': self.yaw_callback}

        self.subscribers = [rospy.Subscriber(e.topic, e.type, self.callbacks[e.topic]) for e in conf.subscribers]

        self.publishers = {e.name: rospy.Publisher(e.topic, e.type, queue_size=1) for e in conf.publishers}

        self.imu_rate = 100
        self.pub_rate = 50
        self.lat = 0
        self.lon = 0
        self.param = param
        print('4')
        #self.lon_controller = PIDLongitudinalController(self.param)
        #self.lat_controller = LateralController(self.param)

    def pos_callback(self, data):
        #global lat, lon
        self.lat = data.lat
        self.lon = data.lon

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
    def run_step(self, goal):
        ct = control()
        ct.override = True
        if goal.gear == PARKING:
            ct.gear = goal.gear
            ct.accel_x = -1
            #ct.accel_x = goal.speed
            ct.theta = 0
            self.publish_controls(ct)
        else:
            ct.gear = goal.gear
            ct.accel_x = 100
            #ct.accel_x = goal.speed
            ct.theta = 0
            self.publish_controls(ct)



    # send stop signal to vehicle
    def emergency_stop(self):
        pass

class VehicleControlServer:
    def __init__(self, name):
        rospy.init_node('controller12')
        self.name = name
        self.server = actionlib.SimpleActionServer(self.name, 
                                                   MoveVehicleAction, 
                                                   execute_cb = self.execute, 
                                                   auto_start=False)
       
        self.controller = Controller(conf)
        self.server.start()
        print('1')
    # get goal from ActionClient(Client)
    def execute(self, goal):
        print('5')
        result = MoveVehicleResult()
        rate = rospy.Rate(50) # 50Hz -> 0.02s CAN rate
        print('2')
        # received goal to stop the vehicle
        if goal.gear == PARKING:
            self.controller.run_step(goal)
            print('3')
        else:
            print('9')
            # run step while vehicle has reached
            # 1개 만족 시 out
            while abs(goal.lat - self.controller.lat) >= 0.000001 or abs(goal.lon - self.controller.lon) >= 0.000001:
                # send Control message[/can_cmd/control] to Vehicle[/can_cmd/vehicle]
                print("lat : %f sub1 : %f sub2 : %f"%(self.controller.lat, goal.lat - self.controller.lat, goal.lon - self.controller.lon))
                #print('gogogogogogo')
                self.controller.run_step(goal)
                rate.sleep()
            #rospy.loginfo("arrived at %f %f", goal.lat, goal.lon)
            print("outyoyoyoyo")
            result.arrived = True
        # send result to ActionClient
        self.server.set_succeeded(result)


if __name__ == '__main__':
    s = VehicleControlServer('vehicle_control')
    rospy.spin()