#!/usr/bin/env python3
import rospy
from can_msg.msg import control, Gear
from can_module import CAN

NEUTRAL = 6
PARKING = 0
DRIVE = 5
REVERSE = 7
FPS = 50

global gear, acc

class Vehicle:
    def __init__(self):
        rospy.init_node('vehicle', anonymous=True)
        rospy.Subscriber('/controller', control, self.callback)
    
        self.can = CAN()
        self.can.get_feedback()
        

        # if self.can.info_1_dict["Gear_shift_Feedback"] == PARKING:
        #     self.initial_gear()

        # elif self.can.info_1_dict["Gear_shift_Feedback"] == REVERSE:
        #     print('r')
        #     self.can.change_gear(PARKING)        
        #     # self.can.change_gear(DRIVE)
        #     # self.can.driving_cmd_dict["Brake_CMD"] = 0

        # elif self.can.info_1_dict["Gear_shift_Feedback"] == NEUTRAL:
        #     print('n')  
        #     # self.can.change_gear(DRIVE)
        #     # self.can.driving_cmd_dict["Brake_CMD"] = 0

        # elif self.can.info_1_dict["Gear_shift_Feedback"] == DRIVE:
        #     self.can.get_feedback()
        #     self.can.change_gear(NEUTRAL)
        #     # self.can.driving_cmd_dict["Brake_CMD"] = 0
        #     # pass
        self.initial_gear()
        
    def initial_gear(self):
        #self.can.get_feedback()
        while self.can.info_1_dict["Gear_shift_Feedback"] == 0:
            self.can.get_feedback()
            print('off')
            #print(self.can.info_1_dict["Gear_shift_Feedback"])
        if self.can.info_1_dict["Gear_shift_Feedback"] == "Parking":
            self.can.change_gear(REVERSE)
            print('1') 
        if self.can.info_1_dict["Gear_shift_Feedback"] == "Reverse":
            self.can.change_gear(NEUTRAL)
            print('2')        
        if self.can.info_1_dict["Gear_shift_Feedback"] == "Neutral":
            self.can.change_gear(DRIVE)
            print('3')
        if self.can.info_1_dict["Gear_shift_Feedback"] == "Driving":
            print('Current Driving')
        
        print("Initial Gear")
          

    # subscribe to Control message then send CAN messages accordingly
    def callback(self, data):            
        # if data.accel_x > 0:
        #     print('this is if')
        #     self.can.driving_cmd_dict["Brake_CMD"] = 0
        #     self.can.driving_cmd_dict["Accel_CMD"] = data.accel_x + 650
        #     #self.can.driving_cmd_dict["Accel_CMD"] = 700
        # else:
        #     print('this is else')
        #     self.can.driving_cmd_dict["Accel_CMD"] = 660
        #     self.can.driving_cmd_dict["Brake_CMD"] = 10000
        #     # self.can.driving_cmd_dict["Brake_CMD"] = -data.accel_x * 1000
        if data.accel_x > 0:
            self.can.driving_cmd_dict["Brake_CMD"] = 0  
            self.can.driving_cmd_dict["Accel_CMD"] = data.accel_x * 150 + 650
        else:
            self.can.driving_cmd_dict["Accel_CMD"] = 650
            self.can.driving_cmd_dict["Brake_CMD"] = -data.accel_x * 400
        self.can.send_control()
        
        # self.can.send_control()
        #self.can.get_feedback()

if __name__ == '__main__':
    try:
        v = Vehicle()
        rospy.spin()

    except KeyboardInterrupt:
        # v.can.driving_cmd_dict["Accel_CMD"] = 650
        # v.can.driving_cmd_dict["Brake_CMD"] = 13000
        # v.can.send_control()
        print("End of Car Control -> Stop Accel")

