from intent import Intent
from traj_map import TrajMap
import numpy as np

__author__ = 'xhou'


class CarGrid:
    def __init__(self):
        pass

    def update(self, in_perc):
        pass


class TrajMgr:
    def __init__(self, traj_map, p):
        self.car_grid = CarGrid(p)
        self.traj_map = traj_map

    def update_perc(self, in_perc):
        self.car_grid = CarGrid(in_perc)

    def gen_speed_profile(self, intent):
        pass

    def gen_traj(self, speed_profile, intent):
        pass

    def eval_traj(self, traj):
        pass


class Traj:
    def __init__(self):
        pass



