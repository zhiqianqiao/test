from collections import deque
import numpy as np
from nav_map import NavMap, LaneExtension

__author__ = 'xhou'


class LaneParser:
    def __init__(self, nav_map, p):
        self.nav_map = nav_map
        self.p = p
        self.lanes = [] * 5

    def parse(self, loc_hist):
        pass


class CarParser:
    def __init__(self, nav_map, p):
        self.nav_map = nav_map
        self.p = p
        self.lane_ext_upper = np.ones(p.cell_num) * p.cell_border_max
        self.lane_ext_lower = -np.ones(p.cell_num) * p.cell_border_max
        self.lane_ext_upper[[1, 7]] = p.cell_border_01
        self.lane_ext_upper[[2, 8]] = -p.cell_border_01
        self.lane_ext_upper[4] = p.cell_border_34
        self.lane_ext_upper[5] = -p.cell_border_34
        self.lane_ext_lower[[0, 6]] = p.cell_border_01
        self.lane_ext_lower[[1, 7]] = -p.cell_border_01
        self.lane_ext_lower[3] = p.cell_border_34
        self.lane_ext_lower[4] = -p.cell_border_34

        self.lane_pool = [set() for i in range(p.cell_num)]
        self.traj_pool = dict()

        self.lane_exts = [] * p.cell_num
        self.frame_cnt = 0

    def update(self, loc, in_perc):
        self.lane_pool = [set() for i in range(self.p.cell_num)]
        self.traj_pool = dict()

        l_loc = self.nav_map.get_par_loc(loc, 'l')
        r_loc = self.nav_map.get_par_loc(loc, 'r')
        ll_loc = self.nav_map.get_par_loc(loc, 'll')
        rr_loc = self.nav_map.get_par_loc(loc, 'rr')
        loc_list = [l_loc, l_loc, l_loc, loc, loc, loc, r_loc, r_loc, r_loc, ll_loc, rr_loc]
        for i in range(self.p.cell_num):
            self.lane_exts[i] = self.nav_map.get_extension(loc_list[i], self.lane_ext_upper[i], self.lane_ext_lower[i])

        for car_id, car_loc in in_perc.iteritems():
            for lane_itr, cur_lane_ext in enumerate(self.lane_exts):
                if cur_lane_ext.contains_loc(car_loc):
                    self.lane_pool[lane_itr].add(car_id)
                    self.traj_pool[car_id] = lane_itr
                    break
