from .state_base import State, defaultdict
__author__ = 'xhou'


class StateACC(State):
    def update_state(self, v_info, perc, in_memory):
        loc = v_info['abs_loc']
        timestamp = v_info['timestamp']

        self.perc_parser.parse(v_info, perc)
        remain_c = self.nav_map.remaining(loc)
        remain_l = self.nav_map.remaining(self.nav_map.get_par_loc(loc, 'l'))
        remain_r = self.nav_map.remaining(self.nav_map.get_par_loc(loc, 'r'))
        if remain_c < self.p.critical_remain_th:
            debug_msg = 'Lane extension too short to peform change lane. Exiting navigation and prepare detour...'
            in_memory.update_memory(State.acc, State.detour, debug_msg, timestamp)
            return in_memory
        recom_l, recom_c, recom_r = self.nav_map.recom(loc)
        if recom_c and (remain_c <= self.p.remain_th):
            debug_msg = 'Termination point within reach. Exiting navigation...'
            in_memory.update_memory(State.acc, State.term, debug_msg, timestamp)
            return in_memory

        state_list = [State.defense]
        scores = dict()
        topdown_scores = defaultdict(float)
        topdown_scores[State.acc] = self.p.acc_bonus

        target_lane = None
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
            if recom_l:
                target_state = State.l_pre_turn
            if recom_r:
                target_state = State.r_pre_turn
            assert target_lane, 'Navigation level lane recommendation bug!'
            state_list.append(target_state)
            topdown_scores[target_state] = (self.p.remain_th - remain_c) / self.p.remain_th * self.p.max_cl_pressure

        for state in state_list:
            scores[state] = self.perc_parser.safety_check(State.acc, state) + topdown_scores[state]

        best_state = max(scores)
        virtual_dist, virtual_speed = self.perc_parser.get_front_vehicle(v_info, best_state)
        traj, _ = self.traj_gen.generate(virtual_dist, virtual_speed, v_info, best_state)

        init_lane = None
        target_lane = None
        debug_msg = 'Best state successfully derived'
        if best_state in State.turn_states:
            direction = 'l' if best_state == State.l_pre_turn else 'r'
            init_lane = self.nav_map.get_extension(self.nav_map.get_par_loc(loc, 'c'))
            target_lane = self.nav_map.get_extension(self.nav_map.get_par_loc(loc, direction))
        in_memory.update_memory(prev_state=State.acc, next_state=State.acc,
                                debug_msg=debug_msg, timestamp=timestamp, scores=scores, traj=traj,
                                init_lane=init_lane, target_lane=target_lane)
        return in_memory
