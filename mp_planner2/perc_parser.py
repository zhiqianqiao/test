import math
import numpy as np

from states.state_base import State
from predictor import Predictor
from lane_cell import LaneCell
from tsmap import Point3d

__author__ = 'xhou'


class PercParser:
    def __init__(self, p):
        self.p = p
        self.nav_map = None
        self.perc = None
        self.timestamp = None

        self.lane_cell = LaneCell()

        self.cell_pool = None
        self.cell_nearest = None
        self.cell_status = None

    def update_map(self, nav_map):
        self.nav_map = nav_map
        self.lane_cell.update_map(nav_map)

    def _get_fv_set(self, planned_state):
        fv_list = list()
        if planned_state not in State.valid_states:
            return list(), 'Ego-state not supported!'
        if planned_state == State.acc:
            for car in self.cell_pool[1]:
                if car['state'][Predictor.r_turn]:
                    fv_list.append(car)
            for car in self.cell_pool[7]:
                if car['state'][Predictor.l_turn]:
                    fv_list.append(car)
            if self.cell_nearest[3]:
                fv_list.append(self.cell_nearest[3])
            msg = 'Front vehicle detected in ACC state. {} car(s) found'.format(len(fv_list))
        if planned_state in {State.l_turn, State.r_turn, State.l_pre_turn, State.r_pre_turn}:
            target_cell_id = 0 if planned_state in {State.l_turn, State.l_pre_turn} else 6
            if self.cell_nearest[target_cell_id]:
                fv_list.append(self.cell_nearest[target_cell_id])
            if self.cell_nearest[3]:
                fv_list.append(self.cell_nearest[3])
            msg = 'Front vehicle detected in turning state. {} car(s) found'.format(len(fv_list))
        return fv_list, msg

    def get_front_vehicle(self, v_info, planned_state):
        loc = v_info['abs_loc']
        fv_list, fv_msg = self._get_fv_set(planned_state)

        min_crash_time = np.inf
        target_car = None
        for car in fv_list:
            rel_speed = car['rel_lv']
            rel_distance = car['rel_l']
            crash_time = -rel_distance / rel_speed
            if 0 < crash_time < min_crash_time:
                target_car = car
                min_crash_time = crash_time

        if target_car:
            return target_car['rel_l'], target_car['rel_lv'] + v_info['speed']
        else:
            # TODO: very slow speed limit
            speed_limit = min(self.nav_map.get_speed_limit(loc), self.p.default_speed_limit)
            return self.p.safe_buffer_time * speed_limit, speed_limit

    def parse(self, v_info, perc, in_memory):
        self.timestamp = in_memory['timestamp']
        self.cell_pool, self.cell_nearest, self.cell_status = self.lane_cell.renew_cells(v_info, perc, in_memory)

    def _panic_check(self):
        if (self.cell_status[4]) or (Predictor.reckless in self.cell_status[3]):
            return True
        return False

    def _eval_safe_condition(self, safe_condition):
        for cell_itr, status in enumerate(self.cell_status):
            if Predictor.reckless in status:
                return False, 'Cell {} has reckless driver!!!!!!'.format(cell_itr)
            if status in safe_condition[cell_itr]:
                return False, 'Cell {} status {}'.format(cell_itr, status)
        return True, ''

    def _safety_pre_check(self, src_state, final_state):
        safe_condition = [set() for i in range(self.p.cell_num)]
        if src_state == State.acc and final_state == State.l_turn:
            safe_condition[0] = {Predictor.underspeed}
            safe_condition[2] = {Predictor.overspeed}
            safe_condition[3] = {Predictor.l_turn, Predictor.underspeed}
            safe_condition[9] = {Predictor.r_turn, Predictor.unknown}
            cond_flag, cond_msg = self._eval_safe_condition(safe_condition)
            if self.cell_pool[1]:
                cond_flag = False
                cond_msg = '\n'.join([cond_msg, 'Cell 1 not empty!'])
            return cond_flag, cond_msg
        if src_state == State.acc and final_state == State.r_turn:
            safe_condition[6] = {Predictor.underspeed}
            safe_condition[8] = {Predictor.overspeed}
            safe_condition[3] = {Predictor.r_turn, Predictor.underspeed}
            safe_condition[10] = {Predictor.l_turn, Predictor.unknown}
            cond_flag, cond_msg = self._eval_safe_condition(safe_condition)
            if self.cell_pool[7]:
                cond_flag = False
                cond_msg = '\n'.join([cond_msg, 'Cell 7 not empty!'])
            return cond_flag, cond_msg
        if src_state == State.l_turn and final_state == State.l_turn:
            safe_condition[0] = {Predictor.underspeed}
            safe_condition[1] = {Predictor.underspeed, Predictor.overspeed}
            safe_condition[9] = {Predictor.r_turn}
            return self._eval_safe_condition(safe_condition)
        if src_state == State.r_turn and final_state == State.r_turn:
            safe_condition[6] = {Predictor.underspeed}
            safe_condition[7] = {Predictor.underspeed, Predictor.overspeed}
            safe_condition[10] = {Predictor.r_turn}
            return self._eval_safe_condition(safe_condition)
        if src_state == State.acc and final_state == State.acc:
            safe_condition[0] = {Predictor.underspeed}
            safe_condition[1] = {Predictor.r_turn}
            safe_condition[7] = {Predictor.l_turn}
            return self._eval_safe_condition(safe_condition)
        return False, 'src -- dst combination not supported!'

    def _density_score(self, cars):
        msg = ''
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
        density = ((max_l - min_l) / (len(cars) - 1))
        score = 1 - (self.p.jam_traffic_density - density) / (self.p.jam_traffic_density - self.p.free_traffic_density)
        score = max(min(1, score), 0)
        return score, msg

    def _rel_diff_score(self, cars):
        if not cars:
            return 1, 'No car found in target lane.'
        rel_time_list = []
        for car in cars:
            rel_time_list.append(abs(car['rel_l'] / car['rel_lv']))
        rel_time = np.min(rel_time_list)

        mid_point = self.p.safe_buffer_time * 2
        slope = self.p.safe_buffer_time / 2
        score = 1 / (1 + math.exp(- slope * (rel_time - mid_point)))
        return score, 'Relative approaching time: {}.'.format(rel_time)

    def safety_check(self, src_state, dst_state):
        if dst_state not in State.valid_states:
            raise 'final_state not supported!!'

        if dst_state == State.defense:
            return self._panic_check()

        if dst_state in {State.l_pre_turn, State.l_turn}:
            dst_state = State.l_turn
        if dst_state in {State.r_pre_turn, State.r_turn}:
            dst_state = State.r_turn

        cond_flag, cond_msg = self._safety_pre_check(src_state, dst_state)
        if not cond_flag:
            return 0.0

        if dst_state == State.l_turn:
            cars = self.cell_pool[0] + self.cell_pool[1] + self.cell_pool[2]
        if dst_state == State.r_turn:
            cars = self.cell_pool[6] + self.cell_pool[7] + self.cell_pool[8]
        if dst_state == State.acc:
            cars = self.cell_pool[3] + self.cell_pool[4] + self.cell_pool[5]
        density_score, density_msg = self._density_score(cars)
        rel_diff_score, rel_diff_msg = self._rel_diff_score(cars)
        return float(density_score + rel_diff_score) / 2
