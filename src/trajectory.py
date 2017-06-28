import numpy as np
from intent import Intent

__author__ = 'yfzhao'


class CarGrid:
    def __init__(self, in_perc):
        pass


class Traj:
    def __init__(self):


    def update_perc(self, in_perc):
        self.car_grid = CarGrid(in_perc)

    def gen_speed_profile(self, intent):
        if intent == Intent.acc:
            front_speed = self.car_grid.g3.velocity
        if intent == Intent.l_turn:
            front_speed = min(self.car_grid.g3.velocity, front_speed = self.car_grid.g0.velocity)
