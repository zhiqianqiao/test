from src.unused.intention import Intention

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

        if intent == Intention.acc:
            acc_speed = self.car_grid.g3.velocity
        if intent == Intention.l_turn:
            acc_speed = min(self.car_grid.g3.velocity, front_speed=self.car_grid.g0.velocity)
        # TODO: right turn and gen speed profile

    def gen_traj(self, intent):
        pass

    def eval_traj(self, traj):
        pass



class Traj:
    def __init__(self):
        pass

