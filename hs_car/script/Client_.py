#!/usr/bin/env python3
from os import path
import rospy
import pandas as pd
import actionlib
from collections import deque
import time
from can_msg.msg import MoveVehicleAction, MoveVehicleGoal

PARKING = 7

class PathPlanner:
    def __init__(self):
        self.path = deque()
        rospy.init_node('action_client')
        # connect to /can_cmd/vehicle_control ActionServer
        self.client = actionlib.SimpleActionClient('vehicle_control', MoveVehicleAction)
        
        self.init_path()

    # get path from csv file -> transform to MoveVehicleGoal objects
    def init_path(self):
        Waypoints = pd.read_csv('/home/nvidia/xavierworkspace/th_catkin_ws/src/hs_car/script/Waypoints2.csv',encoding='utf-8')
        temp = pd.DataFrame(Waypoints)
        for i in range(0,238) :
            #print(temp.at[i,'lat'])
            MoveVehicleGoal.lat= temp.at[i,'lat']
            MoveVehicleGoal.lon= temp.at[i,'lon']
            MoveVehicleGoal.speed= temp.at[i,'speed']
            MoveVehicleGoal.gear= temp.at[i,'gear']
            self.path.append(MoveVehicleGoal)
            # print(self.path[0].lat)
            
        
        print(self.path.popleft().lat)
        print('8')
    # send one goal at a time to ActionServer
    def run(self):
        while self.path:
            print('6')
            self.client.wait_for_server()
            print('7')
            rospy.loginfo('sending goal ...')
            self.client.send_goal(self.path.popleft())
            # hold while waiting for result
            self.client.wait_for_result()
        
        # if path has ended
        else:
            parking = MoveVehicleGoal()
            parking.gear = PARKING

            parking.lat = 0 # default value
            parking.lon = 0 # default value

            parking.speed = 0
            self.client.send_goal(parking)
            
if __name__ == "__main__":
    try:
        pp = PathPlanner()
        pp.run()

    
    except rospy.ROSInterruptException as e:
        print('something wrong with action client:', e)

