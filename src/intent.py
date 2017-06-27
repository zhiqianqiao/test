from collections import deque
import numpy as np
from traj import Traj
from nav_map import NavMap
__author__ = 'xhou'


class Intent:
    l_turn, acc, r_turn, term, emergency, detour = range(6)


class Intention:
    critical_case = {Intent.emergency, Intent.term, Intent.detour}
    valid_case = {Intent.l_turn, Intent.r_turn, Intent.acc}

    def __init__(self, loc, in_perc, p):
        self.nav_map = NavMap()
        self.loc = loc
        self.perc = in_perc
        self.p = p

    def nav_ponder(self):
        if not self.perc.valid():
            return {Intent.emergency}

        remain_th = self.p.remain_th
        remain_c_val = self.nav_map.remaining(self.loc)
        if remain_c_val < self.p.critical_remain_th:
            return {Intent.detour}

        remain_c = remain_c_val >= remain_th
        remain_l = self.nav_map.remaining(self.nav_map.get_par_loc(self.loc, 'l')) >= remain_th
        remain_r = self.nav_map.remaining(self.nav_map.get_par_loc(self.loc, 'r')) >= remain_th
        recom_l, recom_c, recom_r = self.nav_map.recom(self.loc)

        if recom_c and (not remain_c):
            return {Intent.term}

        intent_set = set()
        if remain_c:
            intent_set.add(Intent.acc)
            if self.perc.front_car_speed < self.p.min_front_speed:
                if remain_l:
                    intent_set.add(Intent.l_turn)
                if remain_r:
                    intent_set.add(Intent.r_turn)
        else:
            if recom_l:
                intent_set.add(Intent.l_turn)
            if recom_r:
                intent_set.add(Intent.r_turn)

        if len(intent_set) == 0:
            KeyError('Logic error while pondering: {}{}{}, {}{}{}'.
                     format(remain_l, remain_c, remain_r, recom_l, recom_c, recom_r))
        return intent_set

    def perc_ponder(self, intent_set):
        if Intent.emergency in intent_set:
            return Intent.emergency, Traj()
        if Intent.detour in intent_set:
            return Intent.detour, Traj()
        if Intent.term in intent_set:
            return Intent.term, Traj()

        safe_intent = set()
        max_case_ind = np.max(np.array(Intention.valid_case))
        safety_score = -np.ones(max_case_ind, 1)
        traj_score = -np.ones(max_case_ind, 1)

        for intent in intent_set:
            cur_safety_score = self.check_safety(intent)
            if cur_safety_score > self.p.safety_score_th:
                safety_score[intent] = cur_safety_score
                safe_intent.add(intent)

        if len(safe_intent) == 0:
            return Intent.acc, self.get_traj(Intent.acc)

        raw_traj_dict = dict()
        best_traj_dict = dict()
        for intent in safe_intent:
            cur_traj_list = self.get_traj(intent)
            cur_scores = self.traj_batch_eval(cur_traj_list)
            raw_traj_dict[intent] = cur_traj_list
            best_traj_dict[intent] = cur_traj_list[np.argmax(cur_scores)]
            traj_score[intent] = cur_scores.max()

        total_score_dict = {}
        for intent in intent_set:
            total_score_dict[intent] = traj_score[intent] + safety_score[intent]
        best_intent = max(total_score_dict.iterkeys(), key=(lambda k: total_score_dict[k]))
        self.prev_traj = best_traj_dict[best_intent]
        return best_intent, best_traj_dict[best_intent]

    def check_safety(self, intent):
        # 1 == safe, 0 == not safe
        if intent == Intent.acc:
            return 1
        if intent == Intent.l_turn:
            ext1 = self.nav_map.get_extension(self.nav_map.get_par_loc(self.loc, 'l'))
            ext2 = self.nav_map.get_extension(self.nav_map.get_par_loc(self.loc, 'll'))
        if intent == Intent.r_turn:
            ext1 = self.nav_map.get_extension(self.nav_map.get_par_loc(self.loc, 'r'))
            ext2 = self.nav_map.get_extension(self.nav_map.get_par_loc(self.loc, 'rr'))
        occupancy_discount = 0
        for car in self.perc.cars:
            ext1_flag = self.nav_map.in_extension(car.loc, ext1)
            ext2_flag = self.nav_map.in_extension(car.loc, ext2)
            if ext1_flag or ext2_flag:
                if car.reckless:
                    return 0
                occupancy_discount += self.car_discount(car, ext1_flag)
        return max(0, 1-occupancy_discount)

    def car_discount(self, car, is_adjacent):
        # lots tuning to do here
        if is_adjacent:
            if car.rel_dist < self.p.ext1_min_distance:
                return 1
        else:
            if car.rel_dist < self.p.ext2_min_distance:
                return 1
        return 0

    def get_traj(self, intent):
        if intent not in {Intent.acc, Intent.l_turn, Intent.r_turn}:
            KeyError('Intent not supported! {}'.format(intent))

        traj_mgr = TrajMgr(self.p)
        traj_mgr.init_context(map_data, self.perc, prev_traj)
        traj_mgr.gen_traj(intent)
        return traj_mgr

    def traj_batch_eval(self, traj_list):
        return []
