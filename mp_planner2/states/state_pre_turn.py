from .state_base import State, defaultdict
__author__ = 'xhou'


class StatePreTurn(State):
    def update_state(self, v_info, perc, in_memory):
        loc = v_info['abs_loc']
        timestamp = v_info['timestamp']
        turn_info = in_memory['turn_info']
        planned_state = State.l_turn if turn_info['direction'] == 'l' else State.r_turn
        prev_state = State.l_pre_turn if turn_info['direction'] == 'l' else State.r_pre_turn

        State.signaling_turn_light(planned_state, self.p.light_freq)

        self.perc_parser.parse(v_info, perc, in_memory)

        if self.nav_map.remaining(loc) < self.p.change_lane_th:
            debug_msg = 'Not enough remaining lane to perform lane changing. Detour!'
            in_memory.update_memory(prev_state, State.detour, debug_msg, timestamp)
            return in_memory

        state_list = [State.defense, planned_state]
        scores = dict()
        topdown_scores = defaultdict(float)
        topdown_scores[planned_state] = self.p.abort_pre_cl_penalty

        for state in state_list:
            scores[state] = self.perc_parser.safety_check(State.acc, state) + topdown_scores[state]
        best_state = max(scores)
        virtual_dist, virtual_speed = self.perc_parser.get_front_vehicle(v_info, best_state)
        traj, _ = self.traj_gen.generate(virtual_dist, virtual_speed, v_info, best_state)
        if best_state == State.defense:
            debug_msg = 'Defense!!!'
            in_memory.update_memory(prev_state, State.defense, debug_msg, timestamp,
                                    traj=traj, v_info=v_info, perc_info=perc)
            return in_memory

        if turn_info['start_time'] + self.p.turn_signaling_iter_num < timestamp:
            debug_msg = 'Go for lane changing.'
            in_memory.update_memory(prev_state, planned_state, debug_msg, timestamp, traj=traj)
            return in_memory
        else:
            debug_msg = 'Flash n wait for lane changing'
            in_memory.update_memory(prev_state, planned_state, debug_msg, timestamp, traj=traj)
            return in_memory


class StateLPreTurn(StatePreTurn):
    def update(self, vehicle_info, perc, msg):
        return super(StatePreTurn, self).update(self, vehicle_info, perc, msg)


class StateRPreTurn(StatePreTurn):
    def update(self, vehicle_info, perc, msg):
        return super(StatePreTurn, self).update(self, vehicle_info, perc, msg)
