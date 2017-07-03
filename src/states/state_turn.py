from src.states.state_base import State
__author__ = 'xhou'


class StateTurn(State):
    def update(self, vehicle_info, perc, msg):
        loc_hist = vehicle_info['loc_hist']
        cur_state = msg['state']

        State.signaling_turn_light(cur_state, self.p.light_freq)

        self.perc_parser.parse(loc_hist, perc)
        cur_state = msg['state']
        cur_score = self.perc_parser.safety_check(cur_state, cur_state)
        if cur_score < self.p.change_lane_interrupt_th:
            msg['state'] = State.defense
            msg['traj'] = []
            msg['target_lane'] = None
            msg['txt'] = 'Lane change interupted. Switching to defensive driving mode...'
            return msg

        virtual_dist, virtual_speed = self.perc_parser.get_virtual_car(vehicle_info, cur_state)
        msg['traj'], is_finished = self.traj_gen.generate(virtual_dist, virtual_speed, vehicle_info, cur_state)
        if is_finished:
            msg['state'] = State.acc
            msg['target_lane'] = None
            msg['txt'] = 'Lane change successful.'
        else:
            msg['txt'] = 'Lane change still in progress...'
        return msg


class StateLTurn(State):
    def update(self, vehicle_info, perc, msg):
        return super(StateLTurn, self).update(self, vehicle_info, perc, msg)


class StateRTurn(State):
    def update(self, vehicle_info, perc, msg):
        return super(StateRTurn, self).update(self, vehicle_info, perc, msg)
