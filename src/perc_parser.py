from collections import deque
from src.states.state_base import State
import numpy as np

__author__ = 'xhou'


class PercParser:
    def __init__(self, nav_map, p):
        self.nav_map = nav_map
        self.p = p

        self.cells = [] * p.cell_num
        self.perc = None

        self.cell_pool = [set() for i in range(p.cell_num)]
        self.traj_pool = dict()

    def init_cells(self, loc):
        p = self.p
        cell_upper = np.ones(p.cell_num) * p.cell_border_max
        cell_lower = -np.ones(p.cell_num) * p.cell_border_max
        cell_upper[[1, 7]] = p.cell_border_01
        cell_upper[[2, 8]] = -p.cell_border_01
        cell_upper[4] = p.cell_border_34
        cell_upper[5] = -p.cell_border_34
        cell_lower[[0, 6]] = p.cell_border_01
        cell_lower[[1, 7]] = -p.cell_border_01
        cell_lower[3] = p.cell_border_34
        cell_lower[4] = -p.cell_border_34

        l_loc = self.nav_map.get_par_loc(loc, 'l')
        r_loc = self.nav_map.get_par_loc(loc, 'r')
        ll_loc = self.nav_map.get_par_loc(loc, 'll')
        rr_loc = self.nav_map.get_par_loc(loc, 'rr')
        loc_list = [l_loc, l_loc, l_loc, loc, loc, loc, r_loc, r_loc, r_loc, ll_loc, rr_loc]

        for i in range(self.p.cell_num):
            self.cells[i] = self.nav_map.get_extension(loc_list[i], cell_upper[i], cell_lower[i])

    def parse(self, loc_hist, in_perc):
        self.perc = in_perc
        loc = loc_hist[-1]
        self.init_cells(loc)

        for car_id, car_loc in in_perc.iteritems():
            for lane_itr, cur_lane_ext in enumerate(self.cells):
                if cur_lane_ext.contains_loc(car_loc):
                    self.cell_pool[lane_itr].add(car_id)
                    self.traj_pool[car_id] = lane_itr
                    break

    def query_car(self):
        pass

    def query_cell(self, lane):
        pass

    def front_car_speed(self):
        pass

    def acc_precheck(self, state):
        loc = self.loc_hist[-1]
        # 1 == safe, 0 == not safe
        if state == State.acc:
            return 1
        if state == State.l_turn:
            ext1 = self.nav_map.get_extension(self.nav_map.get_par_loc(loc, 'l'))
            ext2 = self.nav_map.get_extension(self.nav_map.get_par_loc(loc, 'll'))
        if state == State.r_turn:
            ext1 = self.nav_map.get_extension(self.nav_map.get_par_loc(loc, 'r'))
            ext2 = self.nav_map.get_extension(self.nav_map.get_par_loc(loc, 'rr'))
        occupancy_discount = 0
        for car in in_perc.cars:
            ext1_flag = self.nav_map.in_extension([car.loc], ext1)
            ext2_flag = self.nav_map.in_extension([car.loc], ext2)
            if ext1_flag or ext2_flag:
                if car.reckless:
                    return 0
                occupancy_discount += self.car_discount(car, ext1_flag)
        return max(0, 1-occupancy_discount)

    def change_lane_check(self):
        pass

    def car_discount(self, car, is_adjacent):
        if is_adjacent:
            if self.p.ext1_clear_lower < car.rel_dist < self.p.ext1_clear_upper:
                return 1
        else:
            if self.p.ext2_clear_lower < car.rel_dist < self.p.ext2_clear_upper:
                return 1
        return 0