#!/usr/bin/env python3

#from importlib.resources import path
from attrdict import AttrDict
from collections import deque
import numpy as np
import pandas as pd
from math import sqrt
from math import radians
#from Client_ import Pathplanner
#import TH_Server

class PIDLongitudinalController:
    def __init__(self, args) -> None:
        self.max_speed = args.max_speed
        self.max_accel = args.max_accel
        self._p = args.longitudinal.p
        self._i = args.longitudinal.i
        self._d = args.longitudinal.d
        self._dt = args.dt
    
        self._buffer = deque(maxlen=10)

    def run_step(self, cur_vel, cur_acc, cmd_vel):
        error = cmd_vel - (cur_vel + cur_acc * self._dt)
        self._buffer.append(error)

        # if len(self._buffer) >= 2:
        #     de = (self._buffer[-1] - self._buffer[-2])/self._dt
        #     ie = sum(self._buffer) * self._dt
        # else:
        #     de = 0
        #     ie = 0

        cmd_acc = (self._p * error) #+ (self._d * de) + (self._i * ie)
        print("command accel: ", np.clip(cmd_acc, -self.max_accel, self.max_accel))
        return np.clip(cmd_acc, -self.max_accel, self.max_accel)