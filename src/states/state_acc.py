from src.states.state_base import State
__author__ = 'xhou'


class StateACC(State):
    def gen_traj(self):
        # TODO: do it
        pass

    def update(self, loc_hist, in_perc, msg):
        msg = {'state': None, 'target_lane': None, 'txt': ''}
        loc = loc_hist[-1]

        remain_th = self.p.remain_th
        remain_c_val = self.nav_map.remaining(loc)
        if remain_c_val < self.p.critical_remain_th:
            msg['state'] = State.detour
            msg['txt'] = 'Lane extension too short to peform change lane. Exiting navigation and prepare detour...'
            return msg

        remain_c = remain_c_val >= remain_th
        remain_l = self.nav_map.remaining(self.nav_map.get_par_loc(loc, 'l')) >= remain_th
        remain_r = self.nav_map.remaining(self.nav_map.get_par_loc(loc, 'r')) >= remain_th
        recom_l, recom_c, recom_r = self.nav_map.recom(loc)

        if recom_c and (not remain_c):
            msg['state'] = State.term
            msg['txt'] = 'Termination point within reach. Exiting navigation...'
            return msg

        state_set = set()
        if remain_c:
            state_set.add(State.acc)
            if self.perc_parser.front_car_stats() < self.p.min_front_speed:
                if remain_l:
                    state_set.add(State.l_turn)
                if remain_r:
                    state_set.add(State.r_turn)
        else:
            if recom_l:
                state_set.add(State.l_turn)
            if recom_r:
                state_set.add(State.r_turn)

        for state in state_set:
            if state not in State.valid_states:
                msg['state'] = State.emergency
                msg['txt'] = 'Unknown critical error. Possible bugs from NavMap. Exiting navigation...'
                return msg

        max_score = 0
        for state in state_set:
            cur_score = self.perc_parser.safety_check(State.acc, state)
            if cur_score > max_score:
                max_score = cur_score
                best_state = state

        change_lane_pressure = 1 - (remain_c_val - self.p.critical_remain_th) /\
                               (self.p.remain_th - self.p.critical_remain_th) * (1 - self.p.min_change_lane_score)
        if max_score < change_lane_pressure:
            msg['state'] = State.acc
            msg['txt'] = 'Change lane not safe, performing fallback plan (ACC)'
            return msg

        if best_state in {State.l_turn, State.r_turn}:
            turn_val = 'l' if best_state == State.l_turn else 'r'
            msg['target_lane'] = self.nav_map.get_extension(self.nav_map.get_par_loc(loc, turn_val))

        msg['state'] = best_state
        msg['txt'] = 'Best state successfully derived'
        return msg
