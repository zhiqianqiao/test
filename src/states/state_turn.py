from src.states.state_base import State
__author__ = 'xhou'


class StateTurn(State):
    def update(self, loc_hist, in_perc, msg):
        self.perc_parser.parse(loc_hist, in_perc)

        cur_score = self.perc_parser.change_lane_check()
        if cur_score < self.p.change_lane_score:
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


class StateLTurn(StateTurn):
    def update(self, loc_hist, in_perc, msg):
        return super(StateLTurn, self).update_generic(self, loc_hist, in_perc, msg)


class StateRTurn(StateTurn):
    def update(self, loc_hist, in_perc, msg):
        return super(StateRTurn, self).update_generic(self, loc_hist, in_perc, msg)
