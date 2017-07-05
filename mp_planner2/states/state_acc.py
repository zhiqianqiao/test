from .state_base import State
__author__ = 'xhou'


class StateACC(State):
    def update_state(self, v_info, perc, in_msg):
        loc = v_info['abs_loc']

        self.perc_parser.parse(v_info, perc)

        # TODO: Panic check
        # if self.perc_parser.panic_check():
        #     msg['state'] = State.defense
        #     msg['txt'] = 'Panic check failed! Emergency!!!'
        #     return msg

        remain_th = self.p.remain_th
        remain_c_val = self.nav_map.remaining(loc)
        if remain_c_val < self.p.critical_remain_th:
            self.msg['state'] = State.detour
            self.msg['txt'] = 'Lane extension too short to peform change lane. Exiting navigation and prepare detour...'
            self.msg['traj'] = []
            return self.msg

        remain_c = remain_c_val >= remain_th
        remain_l = self.nav_map.remaining(self.nav_map.get_par_loc(loc, 'l')) >= remain_th
        remain_r = self.nav_map.remaining(self.nav_map.get_par_loc(loc, 'r')) >= remain_th
        recom_l, recom_c, recom_r = self.nav_map.recom(loc)

        if recom_c and (not remain_c):
            self.msg['state'] = State.term
            self.msg['txt'] = 'Termination point within reach. Exiting navigation...'
            self.msg['traj'] = []
            return self.msg

        state_list = list()
        if remain_c:
            state_list.append(State.acc)
            # if self.perc_parser.front_car_stats() < self.p.min_front_speed:
            if True:
                if remain_l:
                    state_list.append(State.l_pre_turn)
                if remain_r:
                    state_list.append(State.r_pre_turn)
        else:
            if recom_l:
                state_list.append(State.l_pre_turn)
            if recom_r:
                state_list.append(State.r_pre_turn)

        for state in state_list:
            if state not in State.valid_states:
                self.msg['state'] = State.emergency
                self.msg['txt'] = 'Unknown critical error. Possible bugs from NavMap. Exiting navigation...'
                self.msg['traj'] = []
                return self.msg

        max_score = -100
        for state in state_list:
            cur_score, _ = self.perc_parser.safety_check(State.acc, state)
            if cur_score > max_score:
                max_score = cur_score
                best_state = state
            if state == State.l_pre_turn:
                self.msg['scores'][State.l_turn] = cur_score
            elif state == State.r_pre_turn:
                self.msg['scores'][State.r_turn] = cur_score
            else:
                self.msg['scores'][state] = cur_score

        if best_state in {State.l_pre_turn, State.r_pre_turn}:
            cl_pressure = (self.p.remain_th - remain_c_val) / self.p.remain_th * self.p.max_cl_pressure
            if max_score + cl_pressure < self.p.change_lane_th:
                best_state = State.acc
                self.msg['state'] = best_state
                self.msg['txt'] = 'Planned for lane changing but not safe enough (score: {}, pressure: {}),' \
                             'performing fallback plan (ACC)'.format(max_score, cl_pressure)
                virtual_dist, virtual_speed = self.perc_parser.get_front_vehicle(v_info, best_state)
                self.msg['traj'], _ = self.traj_gen.generate(virtual_dist, virtual_speed, v_info, best_state)
                self.msg['target_lane'] = None
                return self.msg

            turn_val = 'l' if best_state == State.l_pre_turn else 'r'
            self.msg['target_lane'] = self.nav_map.get_extension(self.nav_map.get_par_loc(loc, turn_val))

        self.msg['state'] = best_state
        self.msg['txt'] = 'Best state successfully derived'
        virtual_dist, virtual_speed = self.perc_parser.get_front_vehicle(v_info, best_state)
        self.msg['traj'], _ = self.traj_gen.generate(virtual_dist, virtual_speed, v_info, best_state)
        return self.msg
