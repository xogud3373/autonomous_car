import cantools
import can
import threading
from pygame import time

class GearStateError(Exception):
    def __init__(self, msg):
        self.msg = msg
    def __str__(self):
        return self.msg


PARKING = 0
DRIVE = 5
NEUTRAL = 6
REVERSE = 7

GEAR_STATE = [PARKING, REVERSE, NEUTRAL, DRIVE]

CONTROL_CMD = 1
DRIVING_CMD = 2

VEHICLE_INFO_1 = 3
VEHICLE_INFO_2 = 4

CYCLE_FPS = 50

dt = 0.02


class CAN:
    def __init__(self, destination = None):
        # CAN
        # self.bus = can.interfaces.pcan.PcanBus(channel='PCAN1_USBBUS1', bitrate=500000)
        self.db = cantools.database.load_file("/home/nvidia/car_control/Santafe_Final.dbc")
        self.bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=500000)
        # self.bus = can.interfaces.pcan.PcanBus(channel='PCAN1_USBBUS1', bitrate=500000)
        self.vehicle_info_1_msg = self.db.get_message_by_name('Vehicle_info_1')
        self.vehicle_info_2_msg = self.db.get_message_by_name('Vehicle_Info_2')
        self.info_1_dict = {"APS_Feedback":0, "Brake_ACT_Feedback":0,"Gear_shift_Feedback":0, 
                               "Steering_angle_Feedback":0, "Switch_state":0}
        self.info_2_dict = {"Override_feedback":0, "Vehicle_Speed":0}
        self.control_cmd_msg = self.db.get_message_by_name('Control_CMD') 
        self.driving_cmd_msg = self.db.get_message_by_name('Driving_CMD')
        self.control_cmd_dict = {'Override':0,'Alive_Count':0,'Angular_Speed_CMD':30}  
        self.driving_cmd_dict = {'Accel_CMD':650,'Brake_CMD':0,'Steering_CMD':0,'Gear_shift_CMD':PARKING} 

        self.destination = destination

        self.max_speed = 7  # m/s 
        self.clock = time.Clock()

        self.cur_dist = 0
        self.initialize()

    def initialize(self):
        self.get_feedback()
        # print(self.info_1_dict["Gear_shift_Feedback"])

        # delay = 0
        # while delay < 100:
        #     self.clock.tick_busy_loop(50)
        #     self.get_feedback()
        # if self.info_1_dict["Gear_shift_Feedback"] != "Parking":
        #     print('no')
        #     raise GearStateError("default gear state must be PARKING!")

    def get_feedback(self):
        th1 = threading.Thread(target=self.get_vehicle_info_1)
        th2 = threading.Thread(target=self.get_vehicle_info_2)

        th1.start()
        th2.start()
    
    def send_control(self):
        # th1 = threading.Thread(target=self.send_control_cmd)
        # th2 = threading.Thread(target=self.send_driving_cmd)

        # th1.start()
        # th2.start()
        self.send_driving_cmd()
        

    def change_gear(self, gear):
        if gear == NEUTRAL: gear_feed = "Neutral"
        elif gear == DRIVE: gear_feed = "Driving"
        elif gear == PARKING: gear_feed = "Parking"
        elif gear == REVERSE: gear_feed = "Reverse"   
        print("changing gear ...")
        delay = 0
        while self.info_1_dict["Gear_shift_Feedback"] != gear_feed:
            self.clock.tick_busy_loop(50)
            # print("gear:", gear, "cur gear: ", self.info_1_dict["Gear_shift_Feedback"])
            self.get_feedback()
        
            if self.info_2_dict["Vehicle_Speed"] <= 1 and self.info_1_dict["Brake_ACT_Feedback"] >= 1000:
                #print("delay : ",delay)
                if delay >= 20:
                    self.driving_cmd_dict["Gear_shift_CMD"] = gear
                    self.get_feedback()
            else:
                # print("ok to change gear")
                self.driving_cmd_dict["Brake_CMD"] = min(self.driving_cmd_dict["Brake_CMD"]+200, 13000)
                #print("break_CMD : %d Feedback : %d Speed : %d"% (self.driving_cmd_dict["Brake_CMD"],self.info_1_dict["Brake_ACT_Feedback"], self.info_2_dict["Vehicle_Speed"]))
                delay = 0
            delay += 1
            self.send_control()

    def get_vehicle_info_1(self):
        try:
            msg = self.bus.recv(0.1)  # wait 0.5 sec to get msg then raise error
            data = self.db.decode_message(msg.arbitration_id, msg.data)
            if msg.arbitration_id == self.vehicle_info_1_msg.frame_id:
                self.info_1_dict = data
        except:
            return 0

    def get_vehicle_info_2(self):
        try:
            msg = self.bus.recv(0.1)  # wait 0.5 sec to get msg then raise error
            data = self.db.decode_message(msg.arbitration_id, msg.data)
            if msg.arbitration_id == self.vehicle_info_2_msg.frame_id:
                self.info_2_dict = data
        except:
            return 0

    def send_control_cmd(self):
        # print("_send_command")
        self.control_cmd_dict['Alive_Count'] = (self.control_cmd_dict['Alive_Count'] + 1) % 256
        data = self.control_cmd_msg.encode(self.control_cmd_dict)
        message = can.Message(arbitration_id=self.control_cmd_msg.frame_id, data=data,is_extended_id=False)
        self.bus.send(message)
    

    def send_driving_cmd(self):
        data = self.driving_cmd_msg.encode(self.driving_cmd_dict)
        message = can.Message(arbitration_id=self.driving_cmd_msg.frame_id, data=data,is_extended_id=False)
        self.bus.send(message)
