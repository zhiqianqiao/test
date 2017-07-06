from .state_base import State, defaultdict
__author__ = 'xhou'


class StateTurn(State):
    def update_state(self, v_info, perc, in_memory):
        loc = v_info['abs_loc']
        timestamp = v_info['timestamp']
        turn_info = in_memory['turn_info']
        cur_state = State.l_turn if turn_info['direction'] == 'l' else State.r_turn

        State.signaling_turn_light(cur_state, self.p.light_freq)
        state_list = [State.defense, cur_state]

        self.perc_parser.parse(v_info, perc, in_memory)
        scores = dict()
        topdown_scores = defaultdict(float)
        topdown_scores[cur_state] = self.p.abort_cl_penalty
        for state in state_list:
            scores[state] = self.perc_parser.safety_check(cur_state, cur_state) + topdown_scores[state]

        best_state = max(scores)
        virtual_dist, virtual_speed = self.perc_parser.get_front_vehicle(v_info, best_state)
        traj, is_finished = self.traj_gen.generate(virtual_dist, virtual_speed, v_info, best_state)
        if best_state == State.defense:
            debug_msg = 'Defense!!!'
            in_memory.update_memory(cur_state, State.defense, debug_msg, timestamp,
                                    traj=traj, v_info=v_info, perc_info=perc)
            return in_memory

        if is_finished:
            debug_msg = 'Change lane finished. Will perform ACC'
            next_state = State.acc
        else:
            debug_msg = 'Lane change still in progress...'
            next_state = cur_state
        in_memory.update_memory(cur_state, next_state, debug_msg, timestamp, traj)
        return in_memory


class StateLTurn(StateTurn):
    def update_state(self, v_info, perc, msg):
        return super(StateLTurn, self).update_state(v_info, perc, msg)


class StateRTurn(StateTurn):
    def update_state(self, v_info, perc, msg):
        return super(StateRTurn, self).update_state(v_info, perc, msg)
