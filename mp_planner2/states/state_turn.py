from .state_base import State
__author__ = 'xhou'


class StateTurn(State):
    def update_state(self, v_info, perc, in_msg):
        cur_state = in_msg['state']

        State.signaling_turn_light(cur_state, self.p.light_freq)
        self.perc_parser.parse(v_info, perc)
        cur_score, _ = self.perc_parser.safety_check(cur_state, cur_state)

        print "StateTurn() says: in turn: ", cur_score, self.p.change_lane_interrupt_th
        if cur_score < self.p.change_lane_interrupt_th:
            # TODO: state defense to be implemented
            self.msg['state'] = State.defense
            self.msg['traj'] = []
            self.msg['target_lane'] = None
            self.msg['txt'] = 'Lane change interupted. Switching to defensive driving mode...'
            return self.msg

        virtual_dist, virtual_speed = self.perc_parser.get_front_vehicle(v_info, cur_state)
        self.msg['traj'], is_finished = self.traj_gen.generate(virtual_dist, virtual_speed, v_info, cur_state)
        if is_finished:
            self.msg['state'] = State.acc
            self.msg['target_lane'] = None
            self.msg['txt'] = 'Lane change successful.'
        else:
            print "in turn, current: ", cur_state
            self.msg['state'] = cur_state
            self.msg['target_lane'] = in_msg['target_lane']
            self.msg['txt'] = 'Lane change still in progress...'
        return self.msg


class StateLTurn(StateTurn):
    def update_state(self, v_info, perc, msg):
        return super(StateLTurn, self).update_state(v_info, perc, msg)


class StateRTurn(StateTurn):
    def update_state(self, v_info, perc, msg):
        return super(StateRTurn, self).update_state(v_info, perc, msg)
