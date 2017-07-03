import math

import numpy as np

from states.state_base import State
from predictor import Predictor

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

    def _get_front_vehicle(self, loc, ego_state):
        speed_limit = self.nav_map.get_speed_limit(loc)
        fv_set = set()

        if ego_state not in State.valid_states:
            return set(), 'Ego-state not supported!'
        if ego_state == State.acc:
            for car in self.cell_pool[1]:
                if car['state'] == Predictor.r_turn:
                    fv_set.add(car)
            for car in self.cell_pool[7]:
                if car['state'] == Predictor.l_turn:
                    fv_set.add(car)
            if self.cell_nearest[3]:
                fv_set.add(self.cell_nearest[3])
            msg = 'Front vehicle in ACC state. {} car(s) found'.format(len(fv_set))
        if ego_state in {State.l_turn, State.r_turn, State.l_pre_turn, State.r_pre_turn}:
            target1 = 1 if ego_state in {State.l_turn, State.l_pre_turn} else 7
            target2 = 0 if ego_state == {State.r_turn, State.r_pre_turn} else 6
            near_car = None
            min_l = np.inf
            for car in set.union(self.cell_pool[target1], {self.cell_nearest[target2], self.cell_nearest[3]}):
                rel_l = car['rel_l']
                if 0 < rel_l < min_l:
                    near_car = car
                    min_l = rel_l
            fv_set = {near_car}
            msg = 'Front vehicle detection turn state.'
        if not fv_set:
            fv_set = {self.get_virtual_car(loc, self.p.safe_distance, speed_limit)}
        return fv_set, msg

    def get_virtual_car(self, vehicle_info, ego_state):
        loc = vehicle_info['loc']
        ego_v = vehicle_info['ego_v']

        fv_set, fv_msg = self._get_front_vehicle(loc, ego_state)
        virtual_speed = np.inf
        virtual_dist = 0
        weight = 0
        for car in fv_set:
            conf = car['state_conf']
            weight += conf
            virtual_dist += car['rel_l'] * conf
            if virtual_speed > car['abs_lv']:
                virtual_speed = car['abs_lv']
        virtual_dist /= weight
        virtual_dist -= self.p.safe_distance * ego_v
        return virtual_dist, virtual_speed

    def parse(self, loc_hist, perc):
        self.perc = perc
        loc = loc_hist[-1]
        self.init_cells(loc)

        self.cell_pool = [set() for i in range(self.p.cell_num)]
        self.cell_nearest = [None] * self.p.cell_num
        self.cell_status = [set() for i in range(self.p.cell_num)]

        for car_id, car in perc.iteritems():
            for cell_itr, cur_lane_ext in enumerate(self.cells):
                car_loc = [car['abs_x'], car['abs_y']]
                if cur_lane_ext.contains_loc(car_loc):
                    self.cell_pool[cell_itr].add(car_id)
                    break

        for cell_itr, cars in self.cell_pool:
            if cell_itr in self.nearest_only:
                n_dist = np.inf
                n_car = None
                for car in cars:
                    if car['rel_l'] < n_dist:
                        n_car = car
                        n_dist = car['rel_l']
                self.cell_nearest[cell_itr] = n_car
                car_set = {n_car}
            else:
                car_set = cars

            for car in car_set:
                self.cell_status[cell_itr].add(car['state'])

    # def panic_check(self):
    #     if (self.cell_status[4]) or (Predictor.reckless in self.cell_status[3]):
    #         return True
    #     return False

    def _eval_safe_condition(self, safe_condition):
        for cell_itr, status in enumerate(self.cell_status):
            if Predictor.reckless in status:
                return False, 'Cell {} has reckless driver!!!!!!'.format(cell_itr)
            if status in safe_condition[cell_itr]:
                return False, 'Cell {} status {}'.format(cell_itr, status)
        return True, ''

    def _safety_pre_check(self, src_state, dst_state):
        safe_condition = [set() for i in self.p.cell_num]
        if src_state == State.acc and dst_state == State.l_turn:
            safe_condition[0] = {Predictor.underspeed}
            safe_condition[2] = {Predictor.overspeed}
            safe_condition[3] = {Predictor.l_turn, Predictor.underspeed}
            safe_condition[9] = {Predictor.r_turn, Predictor.unknown}
            cond_flag, cond_msg = self._eval_safe_condition(safe_condition)
            if self.cell_pool[1]:
                cond_flag = False
                cond_msg = '\n'.join([cond_msg, 'Cell 1 not empty!'])
            return cond_flag, cond_msg
        if src_state == State.acc and dst_state == State.r_turn:
            safe_condition[6] = {Predictor.underspeed}
            safe_condition[8] = {Predictor.overspeed}
            safe_condition[3] = {Predictor.r_turn, Predictor.underspeed}
            safe_condition[10] = {Predictor.l_turn, Predictor.unknown}
            cond_flag, cond_msg = self._eval_safe_condition(safe_condition)
            if self.cell_pool[7]:
                cond_flag = False
                cond_msg = '\n'.join([cond_msg, 'Cell 7 not empty!'])
            return cond_flag, cond_msg
        if src_state == State.l_turn and dst_state == State.l_turn:
            safe_condition[0] = {Predictor.underspeed}
            safe_condition[1] = {Predictor.underspeed, Predictor.overspeed}
            safe_condition[9] = {Predictor.r_turn}
            return self._eval_safe_condition(safe_condition)
        if src_state == State.r_turn and dst_state == State.r_turn:
            safe_condition[6] = {Predictor.underspeed}
            safe_condition[7] = {Predictor.underspeed, Predictor.overspeed}
            safe_condition[10] = {Predictor.r_turn}
            return self._eval_safe_condition(safe_condition)
        if src_state == State.acc and dst_state == State.acc:
            safe_condition[0] = {Predictor.underspeed}
            safe_condition[1] = {Predictor.r_turn}
            safe_condition[7] = {Predictor.l_turn}
            return self._eval_safe_condition(safe_condition)
        return False, 'src -- dst combination not supported!'

    def _density_score(self, cars):
        if len(cars) < 2:
            score = 1
            msg = '1 or less car appeared.'
            return score, msg
        min_l = max_l = 0
        for car in cars:
            rel_l = car['rel_l']
            if rel_l > max_l:
                max_l = rel_l
            if rel_l < min_l:
                min_l = rel_l
        target_traffic_density = ((max_l - min_l) / (len(cars) - 1)) / self.p.normal_traffic_density

        if target_traffic_density <= 1:
            score = 0
            msg = 'Traffic Jam in target lane.'
            return score, msg
        if target_traffic_density > (self.p.free_traffic_density / self.p.normal_traffic_density):
            score = 1
            msg = 'Free Traffic in target lane.'
            return score, msg
        else:
            score = math.log1p(target_traffic_density - self.p.free_traffic_density / self.p.normal_traffic_density)
            msg = 'Normal Traffic in target lane.'
            return score, msg

    def _rel_diff_score(self, cars):
        if not cars:
            return 1, 'No car found in target lane.'
        rel_time_list = []
        for car in cars:
            rel_time_list.append(abs(car['rel_l'] / car['rel_speed']))
        rel_time = np.min(rel_time_list)

        mid_point = self.p.safe_distance * 2
        slope = self.p.safe_distance / 2
        score = 1 / (1 + math.exp(- slope * (rel_time - mid_point)))
        return score, 'Relative approaching time: {}.'.format(rel_time)

    def safety_check(self, src_state, dst_state):
        cond_flag, cond_msg = self._safety_pre_check(src_state, dst_state)
        if not cond_flag:
            return 0, cond_msg

        if dst_state == State.l_turn:
            cars = set.union(self.cell_pool[0], self.cell_pool[1], self.cell_pool[2])
        if dst_state == State.r_turn:
            cars = set.union(self.cell_pool[6], self.cell_pool[7], self.cell_pool[8])
        if dst_state == State.acc:
            cars = set.union(self.cell_pool[3], self.cell_pool[4], self.cell_pool[5])
        density_score, density_msg = self._density_score(cars)
        rel_diff_score, rel_diff_msg = self._rel_diff_score(cars)

        return (density_score + rel_diff_msg) / 2, '\n'.join([density_msg, rel_diff_msg])