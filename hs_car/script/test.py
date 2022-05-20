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

@dataclass
class Temp_gps:
    lat:float = None
    lon:float = None
    speed:float = None
    gear:int = None

class PathPlanner:
    def __init__(self):
        self.path = deque()
        self.temp = Temp_gps()
        rospy.init_node('action_client')
        # connect to /can_cmd/vehicle_control ActionServer
        self.client = actionlib.SimpleActionClient('vehicle_control', MoveVehicleAction)
        
        self.init_path()

    # get path from csv file -> transform to MoveVehicleGoal objects
    def init_path(self):
        Waypoints = pd.read_csv('/home/nvidia/xavierworkspace/th_catkin_ws/src/hs_car/script/Waypoints2.csv',encoding='utf-8')
        temp = pd.DataFrame(Waypoints)
        
        di = {}
        li = []
        for i in range(0,238) :
            message_goal = MoveVehicleGoal()
            # print(temp.at[i,'lat'])
            # print(temp.at[i,'lon'])
            # print(temp.at[i,'speed'])
            # print(temp.at[i,'gear'])
            
            #di["lat"]=temp.at
            if i == 0:
                message_goal.lat= temp.at[i,'lat']
                message_goal.lon= temp.at[i,'lon']
                message_goal.speed= temp.at[i,'speed']
                message_goal.gear= temp.at[i,'gear']
                self.path.appendleft(message_goal)
            else:
                message_goal.lat= temp.at[i,'lat']
                message_goal.lon= temp.at[i,'lon']
                message_goal.speed= temp.at[i,'speed']
                message_goal.gear= temp.at[i,'gear']
                self.path.append(message_goal)
            
            # print()
            # # self.temp.lat= temp.at[i,'lat']
            # # self.temp.lon= temp.at[i,'lon']
            # # self.temp.speed= temp.at[i,'speed']
            # # self.temp.gear= temp.at[i,'gear']
            # # print(self.temp.lat)
            # # print(self.temp.lon)
            # # print(self.temp.speed)
            # # print(self.temp.gear)
            # li.append([self.temp])
            # print("i = ",i)
            # print("bye : ",li)
            # if i > 5:
            #     # print(li[i])
            #     # print(li[i-1])
            #     break

        # print("hi : ",li[0])
        # print("hi : ",li[1])
        # print("hi : ",li[2])
        # print("hi : ",li[3])
        # print("hi : ",li[4])

        d = self.path.popleft()
        print(d.lat)
        d = self.path.popleft()
        print(d.lat)
        #print(self.path.popleft().lat)
        #print(self.path.pop().lat)
            
            

        #print(self.path.popleft().lat)
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
        #pp.run()

    
    except rospy.ROSInterruptException as e:
        print('something wrong with action client:', e)

