from src.states.state_base import State
from predictor import Predictor
import numpy as np

__author__ = 'xhou'


class PercParser:
    def __init__(self, nav_map, p):
        self.nav_map = nav_map
        self.p = p

        self.cells = [] * p.cell_num
        self.perc = None

        self.cell_pool = []
        self.cell_nearest = []
        self.cell_status = []

        self.nearest_only = {0, 2, 3, 5, 6, 8}

    def init_cells(self, loc):
        p = self.p
        cell_upper = np.ones(p.cell_num) * p.cell_border_1
        cell_lower = np.ones(p.cell_num) * p.cell_border_6

        cell_upper[[1, 7, 9, 10]] = p.cell_border_2
        cell_upper[4] = p.cell_border_3
        cell_upper[[2, 5, 8]] = p.cell_border_5

        cell_lower[[0, 6]] = p.cell_border_2
        cell_lower[3] = p.cell_border_3
        cell_lower[[9, 10]] = p.cell_border_4
        cell_lower[[1, 4, 7]] = p.cell_border_5

        l_loc = self.nav_map.get_par_loc(loc, 'l')
        r_loc = self.nav_map.get_par_loc(loc, 'r')
        ll_loc = self.nav_map.get_par_loc(loc, 'll')
        rr_loc = self.nav_map.get_par_loc(loc, 'rr')
        loc_list = [l_loc, l_loc, l_loc, loc, loc, loc, r_loc, r_loc, r_loc, ll_loc, rr_loc]

        for i in range(self.p.cell_num):
            self.cells[i] = self.nav_map.get_extension(loc_list[i], cell_upper[i], cell_lower[i])

    def _get_front_vehicle(self, loc):
        speed_limit = self.nav_map.get_speed_limit(loc)
        fv_set = set()

        for car in self.cell_pool[1]:
            if car.state == Predictor.r_turn:
                fv_set.add(car)
        for car in self.cell_pool[7]:
            if car.state == Predictor.l_turn:
                fv_set.add(car)
        if self.cell_nearest[3]:
            fv_set.add(self.cell_nearest[3])

        if not fv_set:
            return {Predictor.add_virtual_car(self.p.safe_distance, speed_limit)}
        return fv_set

    def get_virtual_wall(self, loc, ego_v):
        fv_set = self._get_front_vehicle(loc)
        wall_speed = np.inf
        wall_dist = 0
        weight = 0
        for car in fv_set:
            conf = car.action_conf
            weight += conf
            wall_dist += car.rel_l * conf
            if wall_speed > car.abs_lv:
                wall_speed = car.abs_lv
        wall_dist /= weight
        wall_dist -= self.p.safe_distance * ego_v
        return wall_dist, wall_speed

    def parse(self, loc_hist, in_perc):
        self.perc = in_perc
        loc = loc_hist[-1]
        self.init_cells(loc)

        self.cell_pool = [set() for i in range(self.p.cell_num)]
        self.cell_nearest = [None] * self.p.cell_num
        self.cell_status = [set() for i in range(self.p.cell_num)]

        # TODO: perception parsing
        for car_id, car_loc in in_perc.iteritems():
            for cell_itr, cur_lane_ext in enumerate(self.cells):
                if cur_lane_ext.contains_loc(car_loc):
                    self.cell_pool[cell_itr].add(car_id)
                    break

        for cell_itr, cars in self.cell_pool:
            if cell_itr in self.nearest_only:
                n_dist = np.inf
                n_car = None
                for car in cars:
                    if car.rel_l < n_dist:
                        n_car = car
                        n_dist = car.rel_l
                self.cell_nearest[cell_itr] = n_car
                car_set = {n_car}
            else:
                car_set = cars

            for car in car_set:
                self.cell_status[cell_itr].add(car.state)

    def panic_check(self):
        if (self.cell_status[4]) or (Predictor.reckless in self.cell_status[3]):
            return True
        return False

    def _safety_ckeck(self, safe_condition):
        for cell_itr, status in enumerate(self.cell_status):
            if Predictor.reckless in status:
                return False, 'Cell {} has reckless driver!!!!!!'.format(cell_itr)
            if status in safe_condition[cell_itr]:
                return False, 'Cell {} status {}'.format(cell_itr, status)
        return True, ''

    def safety_check(self, src_state, dst_state):
        # TODO: calculate a value for each state
        safe_condition = [set() for i in self.p.cell_num]
        if src_state == State.acc and dst_state == State.l_turn:
            safe_condition[0] = {Predictor.brake}
            safe_condition[2] = {Predictor.accel}
            safe_condition[3] = {Predictor.l_turn, Predictor.brake}
            safe_condition[9] = {Predictor.r_turn, Predictor.unknown}
            cond_flag, cond_msg = self._safety_ckeck(safe_condition)
            if self.cell_pool[1]:
                cond_flag = False
                cond_msg += '   Cell 1 not empty!'
            return cond_flag, cond_msg
        if src_state == State.acc and dst_state == State.r_turn:
            safe_condition[6] = {Predictor.brake}
            safe_condition[8] = {Predictor.accel}
            safe_condition[3] = {Predictor.r_turn, Predictor.brake}
            safe_condition[10] = {Predictor.l_turn, Predictor.unknown}
            cond_flag, cond_msg = self._safety_ckeck(safe_condition)
            if self.cell_pool[7]:
                cond_flag = False
                cond_msg += '   Cell 7 not empty!'
            return cond_flag, cond_msg
        if src_state == State.l_turn and dst_state == State.l_turn:
            safe_condition[0] = {Predictor.brake}
            safe_condition[1] = {Predictor.brake, Predictor.accel}
            safe_condition[9] = {Predictor.r_turn}
            return self._safety_ckeck(safe_condition)
        if src_state == State.r_turn and dst_state == State.r_turn:
            safe_condition[6] = {Predictor.brake}
            safe_condition[7] = {Predictor.brake, Predictor.accel}
            safe_condition[10] = {Predictor.r_turn}
            return self._safety_ckeck(safe_condition)
        if src_state == State.acc and dst_state == State.acc:
            safe_condition[0] = {Predictor.brake}
            safe_condition[1] = {Predictor.r_turn}
            safe_condition[7] = {Predictor.l_turn}
            return self._safety_ckeck(safe_condition)
        return False, 'src -- dst combination not supported!'
