
from importlib.resources import path
from attrdict import AttrDict
from collections import deque
import numpy as np
import pandas as pd
from math import sqrt
from math import radians
from Client_ import Pathplanner
import TH_Server

class PIDLongitudinalController:
    def __init__(self, args) -> None:
        self.max_speed = args.max_speed
        self.max_accel = args.max_accel
        self._p = args.longitudinal.p
        self._i = args.longitudinal.i
        self._d = args.longitudinal.d
        self._dt = args.longitudianl.dt
    
        self._buffer = deque(maxlen=10)

    def run_step(self, cur_vel, cur_acc, cmd_vel):
        error = cmd_vel - (cur_vel + cur_acc * self._dt)
        self._buffer.append(error)

        if len(self._buffer) >= 2:
            de = (self._buffer[-1] - self._buffer[-2])/self._dt
            ie = sum(self._buffer) * self._dt
        else:
            de = 0
            ie = 0

        cmd_acc = (self._p * error) + (self._d * de) + (self._i * ie)
        
        return np.clip(cmd_acc, -self.max_accel, self.max_accel)
    
class LateralController :
    # parameters
    dt = 0.02

    k = 0.5  # control gain

    class VehicleModel(object):
        def __init__(self, x=Pathplanner.path.lat[0], y=Pathplanner.path.lon[1], yaw =Server_.yaw, v=0):
            self.x = Pathplanner.path.lat[0]
            self.y = Pathplanner.path.lon[1]
            self.yaw = Server_.yaw
            self.v = v

            self.max_steering = np.radians(30)

        def update(self, steer, cur_vel):
            steer = np.clip(steer, -self.max_steering, self.max_steering)
            self.x += self.v * np.cos(self.yaw) * dt
            self.y += self.v * np.sin(self.yaw) * dt
            self.yaw += self.v /2 * np.tan(steer) * dt
            self.yaw = self.yaw % (2.0 * np.pi)
            self.v = cur_vel


    def normalize_angle(angle):
        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi

        return angle


    def stanley_control(x, y, yaw, v, target_x, target_y, map_yaws):
        # find nearest point
        min_dist = 0.0002337
        min_index = 0
        n_points = len(target_x)

        front_x = x
        front_y = y

        for i in range(n_points):
            dx = front_x-target_x[i]
            dy = front_y-target_y[i]

            dist = np.sqrt(dx * dx + dy * dy)
            if dist < min_dist:
                min_dist = dist
                min_index = i

        # compute cte at front axle
        target_x2= target_x[min_index]
        target_y2 = target_y[min_index]
        map_yaw = map_yaws[min_index]
        dx = target_x2 - front_x
        dy = target_y2 - front_y

        perp_vec = [np.cos(yaw + np.pi/2), np.sin(yaw + np.pi/2)]
        cte = np.dot([dx, dy], perp_vec)

        # control law
        yaw_term = normalize_angle(map_yaw - yaw)
        cte_term = np.arctan2(k*cte, v)

        # steering
        steer = yaw_term + cte_term
        return steer


    # map
    target_y =[]
    target_x =[]
    map_yaws =[]
    Waypoints = pd.read_csv('/home/glad/바탕화면/good/Waypoints2.csv',encoding='utf-8')
    temp = pd.DataFrame(Waypoints)
    for i in range(0,238) : 
            lat=temp.at[i,'lat']
            lon=temp.at[i,'lon']
            target_x.append(lon)
            target_y.append(lat)
            target_speed= temp.at[i,'speed']
            gear= temp.at[i,'gear']
            map_yaws.append(np.arctan(target_x[i+1]-target_x[i],target_y[i+1]-target_y[i+1]))
            

