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

    def __init__(self, p):
        self.nav_map = NavMap()
        self.p = p

    def nav_ponder(self, loc, in_perc):
        self.loc_hist.append(loc)
        if not in_perc.valid():
            return {Intent.emergency}

        remain_th = self.p.remain_th
        remain_c_val = self.nav_map.remaining(loc)
        if remain_c_val < self.p.critical_remain_th:
            return {Intent.detour}

        remain_c = remain_c_val >= remain_th
        remain_l = self.nav_map.remaining(self.nav_map.get_par_loc(loc, 'l')) >= remain_th
        remain_r = self.nav_map.remaining(self.nav_map.get_par_loc(loc, 'r')) >= remain_th
        recom_l, recom_c, recom_r = self.nav_map.recom(loc)

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

    def perc_ponder(self, loc, in_perc, intent_set):
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
            cur_safety_score = self.check_safety(loc, in_perc, intent)
            if cur_safety_score > self.p.safety_score_th:
                safety_score[intent] = cur_safety_score
                safe_intent.add(intent)

        if len(safe_intent) == 0:
            return Intent.acc, self.get_traj(in_perc, Intent.acc)

        raw_traj_dict = dict()
        best_traj_dict = dict()
        for intent in safe_intent:
            cur_traj_list = self.get_traj(in_perc, intent)
            cur_scores = self.traj_batch_eval(cur_traj_list, in_perc)
            raw_traj_dict = cur_traj_list
            best_traj_dict[intent] = cur_traj_list[np.argmax(cur_scores)]
            traj_score[intent] = cur_scores.max()

    def check_safety(self, loc, in_perc, intent):
        if intent == Intent.acc:
            return 0
        if intent == Intent.l_turn:
            ext1 = self.nav_map.get_extension(self.nav_map.get_par_loc(loc, 'l'))
            ext2 = self.nav_map.get_extension(self.nav_map.get_par_loc(loc, 'll'))
        if intent == Intent.r_turn:
            ext1 = self.nav_map.get_extension(self.nav_map.get_par_loc(loc, 'r'))
            ext2 = self.nav_map.get_extension(self.nav_map.get_par_loc(loc, 'rr'))
        for car in in_perc.cars:
            ext1_flag = self.nav_map.in_extension(car.loc, ext1)
            ext2_flag = self.nav_map.in_extension(car.loc, ext2)
            if ext1_flag or ext2_flag:
                if car.reckless:
                    return 0
                if ext1_flag:
                    pass











    def get_traj(self, in_perc, intent):
        if intent not in {Intent.acc, Intent.l_turn, Intent.r_turn}:
            KeyError('Intent not supported! {}'.format(intent))
        return []

    def traj_batch_eval(self, traj_list, in_perc):
        return []


    def get_score(self, intention, in_perc):
        if intention == Intent.acc:

            in_perc