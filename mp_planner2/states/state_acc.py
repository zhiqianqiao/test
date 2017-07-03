from .state_base import State
__author__ = 'xhou'


class StateACC(State):
    def update(self, vehicle_info, perc, msg):
        msg = {'state': None, 'target_lane': None, 'txt': ''}
        loc_hist = vehicle_info['loc_hist']
        ego_v = vehicle_info['ego_v']
        loc = loc_hist[-1]

        self.perc_parser.parse(loc_hist, perc)

        # TODO: Panic check
        # if self.perc_parser.panic_check():
        #     msg['state'] = State.defense
        #     msg['txt'] = 'Panic check failed! Emergency!!!'
        #     return msg

        remain_th = self.p.remain_th
        remain_c_val = self.nav_map.remaining(loc)
        if remain_c_val < self.p.critical_remain_th:
            msg['state'] = State.detour
            msg['txt'] = 'Lane extension too short to peform change lane. Exiting navigation and prepare detour...'
            msg['traj'] = []
            return msg

        remain_c = remain_c_val >= remain_th
        remain_l = self.nav_map.remaining(self.nav_map.get_par_loc(loc, 'l')) >= remain_th
        remain_r = self.nav_map.remaining(self.nav_map.get_par_loc(loc, 'r')) >= remain_th
        recom_l, recom_c, recom_r = self.nav_map.recom(loc)

        if recom_c and (not remain_c):
            msg['state'] = State.term
            msg['txt'] = 'Termination point within reach. Exiting navigation...'
            msg['traj'] = []
            return msg

        state_set = set()
        if remain_c:
            state_set.add(State.acc)
            if self.perc_parser.front_car_stats() < self.p.min_front_speed:
                if remain_l:
                    state_set.add(State.l_pre_turn)
                if remain_r:
                    state_set.add(State.r_pre_turn)
        else:
            if recom_l:
                state_set.add(State.l_pre_turn)
            if recom_r:
                state_set.add(State.r_pre_turn)

        for state in state_set:
            if state not in State.valid_states:
                msg['state'] = State.emergency
                msg['txt'] = 'Unknown critical error. Possible bugs from NavMap. Exiting navigation...'
                msg['traj'] = []
                return msg

        max_score = 0
        for state in state_set:
            cur_score = self.perc_parser.safety_check(State.acc, state)
            if cur_score > max_score:
                max_score = cur_score
                best_state = state

        if best_state in {State.l_pre_turn, State.r_pre_turn}:
            cl_pressure = (self.p.remain_th - remain_c_val) / self.p.remain_th * self.p.max_cl_pressure
            if max_score + cl_pressure < self.p.change_lane_th:
                best_state = State.acc
                msg['state'] = best_state
                msg['txt'] = 'Planned for lane changing but not safe enough (score: {}, pressure: {}),' \
                             'performing fallback plan (ACC)'.format(max_score, cl_pressure)
                virtual_dist, virtual_speed = self.perc_parser.get_virtual_car(vehicle_info, best_state)
                msg['traj'], _ = self.traj_gen.generate(virtual_dist, virtual_speed, vehicle_info, best_state)
                return msg

            turn_val = 'l' if best_state == State.l_pre_turn else 'r'
            msg['target_lane'] = self.nav_map.get_extension(self.nav_map.get_par_loc(loc, turn_val))

        msg['state'] = best_state
        msg['txt'] = 'Best state successfully derived'
        virtual_dist, virtual_speed = self.perc_parser.get_virtual_car(vehicle_info, best_state)
        msg['traj'], _ = self.traj_gen.generate(virtual_dist, virtual_speed, vehicle_info, best_state)
        return msg
