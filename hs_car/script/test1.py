#!/usr/bin/env python3
from email import message
from os import path
import rospy
import pandas as pd
import actionlib
from collections import deque
from can_msg.msg import MoveVehicleAction, MoveVehicleGoal
from dataclasses import dataclass

PARKING = 7

class PathPlanner:
    def __init__(self):
        self.path = deque()

        self.message_goal = MoveVehicleGoal()

        Waypoints = pd.read_csv('/home/nvidia/xavierworkspace/th_catkin_ws/src/hs_car/script/Waypoints2.csv',encoding='utf-8')
        self.temp = pd.DataFrame(Waypoints)
        rospy.init_node('action_client')

        # connect to /can_cmd/vehicle_control ActionServer
        self.client = actionlib.SimpleActionClient('vehicle_control', MoveVehicleAction)
        
        self.init_path()

    # get path from csv file -> transform to MoveVehicleGoal objects
    def init_path(self):
        # self.message_goal.lat= self.temp.at[0,'lat']
        # self.message_goal.lon= self.temp.at[0,'lon']
        # self.message_goal.speed= self.temp.at[0,'speed']
        # self.message_goal.gear= self.temp.at[0,'gear']
        # self.path.append(self.message_goal)


        # for i in range(0,10):
        #     self.message_goal.lat= self.temp.at[i,'lat']
        #     self.message_goal.lon= self.temp.at[i,'lon']
        #     self.message_goal.speed= self.temp.at[i,'speed']
        #     self.message_goal.gear= self.temp.at[i,'gear']
        #     print(self.message_goal.lat)
        #     print(self.message_goal.lon)
        #     print(self.message_goal.speed)
        #     print(self.message_goal.gear)
        
        
        print('8')
    # send one goal at a time to ActionServer
    def run(self):
        while self.path:
            for i in range(0, 2):
                print('6')
                self.client.wait_for_server()
                print('7')
                rospy.loginfo('sending goal ...')
                self.message_goal.lat= self.temp.at[i,'lat']
                self.message_goal.lon= self.temp.at[i,'lon']
                self.message_goal.speed= self.temp.at[i,'speed']
                self.message_goal.gear= self.temp.at[i,'gear']
                print(self.message_goal.lat)
                self.client.send_goal(self.message_goal)
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

    # except KeyboardInterrupt:
    #     print("exit")
    except rospy.ROSInterruptException as e:
        print('something wrong with action client:', e)

# end 
# lat: 35.2304347403
# lon: 126.840595177

# mid
#lat: 35.2303589371
#lon: 126.84172294

