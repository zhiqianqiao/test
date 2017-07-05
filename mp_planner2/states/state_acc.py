from .state_base import State
__author__ = 'xhou'


class StateACC(State):
    def _sanity_check(self, state_list, timestamp):
        sanity = True
        if len(state_list) == 0:
            sanity = False
            debug_msg = 'Failed to perfom any action. Exiting'
        for state in state_list:
            if state not in State.valid_states:
                sanity = False
                debug_msg = 'Unknown critical error. Possible bugs from NavMap. Exiting navigation...'
        if len(debug_msg) > 0:
            self.memory.update_memory(prev_state=State.acc, next_state=State.emergency,
                                      debug_msg=debug_msg, timestamp=timestamp)
        return sanity

    def update_state(self, v_info, perc, in_memory):
        loc = v_info['abs_loc']
        timestamp = v_info['timestamp']

        self.perc_parser.parse(v_info, perc)
        if self.perc_parser.panic_check():
            debug_msg = 'Panic check failed! Defense!'
            in_memory.update_memory(State.acc, State.defense, debug_msg, timestamp,
                                    v_info=v_info, perc_info=perc)
            return in_memory

        remain_c = self.nav_map.remaining(loc)
        remain_l = self.nav_map.remaining(self.nav_map.get_par_loc(loc, 'l'))
        remain_r = self.nav_map.remaining(self.nav_map.get_par_loc(loc, 'r'))
        if remain_c < self.p.critical_remain_th:
            debug_msg = 'Lane extension too short to peform change lane. Exiting navigation and prepare detour...'
            in_memory.update_memory(State.acc, State.detour, debug_msg, timestamp)
            return in_memory
        recom_l, recom_c, recom_r = self.nav_map.recom(loc)
        if recom_c and (not remain_c):
            debug_msg = 'Termination point within reach. Exiting navigation...'
            in_memory.update_memory(State.acc, State.term, debug_msg, timestamp)
            return in_memory

        state_list = list()
        cl_prs = 0
        if remain_c > self.p.remain_th:
            state_list.append(State.acc)
            # TODO: implement minimal lane changing policy
            # if self.perc_parser.front_car_stats() < self.p.min_front_speed:
            if True:
                if remain_l > self.p.remain_th:
                    state_list.append(State.l_pre_turn)
                if remain_r > self.p.remain_th:
                    state_list.append(State.r_pre_turn)
        else:
            cl_prs = (self.p.remain_th - remain_c) / self.p.remain_th * self.p.max_cl_pressure
            if recom_l:
                state_list.append(State.l_pre_turn)
            if recom_r:
                state_list.append(State.r_pre_turn)

        if not self._sanity_check(state_list, timestamp):
            return in_memory

        scores = dict()
        for state in state_list:
            scores[state], diagnose_msg = self.perc_parser.safety_check(State.acc, state)
            if state in State.turn_states:
                scores[state] = min(1.2, scores[state] + cl_prs)

        best_state = max(scores)
        init_lane = target_lane = None
        debug_msg = 'Best state successfully derived'
        if best_state in State.turn_states:
            if scores[best_state] < self.p.change_lane_th:
                best_state = State.acc
                debug_msg = 'Replace lane changing (score: {}, pressure: {}) by ACC'.format(scores[best_state], cl_prs)
            else:
                direction = 'l' if best_state == State.l_pre_turn else 'r'
                init_lane = self.nav_map.get_extension(self.nav_map.get_par_loc(loc, 'c'))
                target_lane = self.nav_map.get_extension(self.nav_map.get_par_loc(loc, direction))

        virtual_dist, virtual_speed = self.perc_parser.get_front_vehicle(v_info, best_state)
        traj, _ = self.traj_gen.generate(virtual_dist, virtual_speed, v_info, best_state)
        in_memory.update_memory(prev_state=State.acc, next_state=State.acc,
                                debug_msg=debug_msg, timestamp=timestamp, scores=scores, traj=traj,
                                init_lane=init_lane, target_lane=target_lane)
        return in_memory
