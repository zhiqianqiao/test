from src.states.state_base import State
__author__ = 'xhou'


class StateTurn(State):
    def update(self, loc_hist, perc, msg):
        self.perc_parser.parse(loc_hist, perc)
        cur_state = msg['state']

        cur_score = self.perc_parser.safety_check(cur_state, cur_state)
        if cur_score < self.p.change_lane_interrupt_th:
            msg['state'] = State.defense
            msg['txt'] = 'Lane change interupted. Switching to defensive driving mode...'
            return msg
        if self.nav_map.in_extension(loc_hist, msg['target_lane']):
            msg['state'] = State.acc
            msg['target_lane'] = None
            msg['txt'] = 'Lane change successful.'
            return msg

        msg['txt'] = 'Lane change still in progress...'
        return msg


class StateLTurn(State):
    def update(self, loc_hist, perc, msg):
        return super(StateLTurn, self).update(self, loc_hist, perc, msg)


class StateRTurn(State):
    def update(self, loc_hist, perc, msg):
        return super(StateRTurn, self).update(self, loc_hist, perc, msg)
