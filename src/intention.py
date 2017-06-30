__author__ = 'xhou'


class Intention:
    l_turn, acc, r_turn, defense, term, emergency, detour = range(7)
    valid_cases = {l_turn, r_turn, acc, defense}
    case_names = {'l_turn': l_turn, 'acc': acc, 'r_turn': r_turn, 'defense': defense,
                  'term': term, 'emergency': emergency, 'detour': detour}

    def __init__(self, nav_map, p):
        self.nav_map = nav_map
        self.p = p
        self.loc_hist = None
        self.perc = None

    def update_acc(self, loc_hist, in_perc):
        self.loc_hist = loc_hist
        loc = loc_hist[-1]
        self.perc = in_perc

        remain_th = self.p.remain_th
        remain_c_val = self.nav_map.remaining(loc)
        if remain_c_val < self.p.critical_remain_th:
            return Intention.detour, \
                   'Lane extension too short to peform change lane. Exiting navigation and prepare detour...'

        remain_c = remain_c_val >= remain_th
        remain_l = self.nav_map.remaining(self.nav_map.get_par_loc(loc, 'l')) >= remain_th
        remain_r = self.nav_map.remaining(self.nav_map.get_par_loc(loc, 'r')) >= remain_th
        recom_l, recom_c, recom_r = self.nav_map.recom(loc)

        if recom_c and (not remain_c):
            return Intention.term, \
                   'Termination point within reach. Exiting navigation...'

        intent_set = set()
        if remain_c:
            intent_set.add(Intention.acc)
            if self.perc.front_car_speed < self.p.min_front_speed:
                if remain_l:
                    intent_set.add(Intention.l_turn)
                if remain_r:
                    intent_set.add(Intention.r_turn)
        else:
            if recom_l:
                intent_set.add(Intention.l_turn)
            if recom_r:
                intent_set.add(Intention.r_turn)

        for intent in intent_set:
            if intent not in Intention.valid_cases:
                return Intention.emergency, \
                       'Unknown critical error. Possible bugs from NavMap. Exiting navigation...'

        max_score = 0
        for intent in intent_set:
            cur_score = self.pre_change_lane_check(intent)
            if cur_score > max_score:
                max_score = cur_score
                best_intent = intent

        if max_score < self.p.pre_change_lane_score:
            return Intention.acc, \
                   'Change lane not safe, performing fallback plan (ACC)'
        return best_intent, 'Best intent successfully derived'

    def update_change_lane(self, loc_hist, in_perc, intent, target_ext):
        self.loc_hist = loc_hist
        self.perc = in_perc
        cur_score = self.change_lane_check(in_perc)
        if cur_score < self.p.change_lane_score:
            return Intention.defense, 'Lane change interupted. Switching to defensive driving mode...'
        if self.nav_map.in_extension(loc_hist, target_ext):
            return Intention.acc, 'Lane change successful.'
        return intent, 'Lane change still in progress...'

    def change_lane_check(self, in_perc):
        return 1

    def pre_change_lane_check(self, intent):
        loc = self.loc_hist[-1]
        # 1 == safe, 0 == not safe
        if intent == Intention.acc:
            return 1
        if intent == Intention.l_turn:
            ext1 = self.nav_map.get_extension(self.nav_map.get_par_loc(loc, 'l'))
            ext2 = self.nav_map.get_extension(self.nav_map.get_par_loc(loc, 'll'))
        if intent == Intention.r_turn:
            ext1 = self.nav_map.get_extension(self.nav_map.get_par_loc(loc, 'r'))
            ext2 = self.nav_map.get_extension(self.nav_map.get_par_loc(loc, 'rr'))
        occupancy_discount = 0
        for car in self.perc.cars:
            ext1_flag = self.nav_map.in_extension([car.loc], ext1)
            ext2_flag = self.nav_map.in_extension([car.loc], ext2)
            if ext1_flag or ext2_flag:
                if car.reckless:
                    return 0
                occupancy_discount += self.car_discount(car, ext1_flag)
        return max(0, 1-occupancy_discount)

    def car_discount(self, car, is_adjacent):
        if is_adjacent:
            if self.p.ext1_clear_lower < car.rel_dist < self.p.ext1_clear_upper:
                return 1
        else:
            if self.p.ext2_clear_lower < car.rel_dist < self.p.ext2_clear_upper:
                return 1
        return 0
