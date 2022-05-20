#!/usr/bin/env python3
from email import message
from os import path
import rospy
import pandas as pd
import actionlib
from collections import deque
from can_msg.msg import MoveVehicleAction, MoveVehicleGoal
from dataclasses import dataclass


NEUTRAL = 6
PARKING = 0
DRIVE = 5
REVERSE = 7
FPS = 50

class PathPlanner:
    def __init__(self):
        self.path = deque()

        

        Waypoints = pd.read_csv('/home/nvidia/xavierworkspace/th_catkin_ws/src/hs_car/script/Waypoints2.csv',encoding='utf-8')
        self.temp = pd.DataFrame(Waypoints)
        rospy.init_node('action_client')

        # connect to /can_cmd/vehicle_control ActionServer
        self.client = actionlib.SimpleActionClient('vehicle_control', MoveVehicleAction)
        
        self.init_path()

    # get path from csv file -> transform to MoveVehicleGoal objects
    def init_path(self):
        message_goal = MoveVehicleGoal()
        message_goal.lat= 35.2303589371
        message_goal.lon= 126.84172294
        message_goal.speed= 0.
        message_goal.gear= 5
        path.append(self.message_goal)

        message_goal = MoveVehicleGoal()
        message_goal.lat= 35.2304347403
        message_goal.lon= 126.840595177
        message_goal.speed= 0.
        message_goal.gear= 5
        path.append(self.message_goal)

    
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
            print('10')
        # if path has ended
        else:
            print('11')
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

