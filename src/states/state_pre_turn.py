from src.states.state_base import State
__author__ = 'xhou'


class StatePreTurn(State):
    def __init__(self, nav_map, perc_parser, p):
        self.nav_map = nav_map
        self.perc_parser = perc_parser
        self.start_time = -1
        self.p = p

    def update(self, loc_hist, perc, msg, time_stamp, direction):
        planned_state = State.l_turn if direction == 'l' else State.r_turn
        self.perc_parser.parse(loc_hist, perc)
        cur_score = self.perc_parser.safety_check(State.acc, planned_state)

        remain_c_val = self.nav_map.remaining(loc_hist[-1])
        cl_pressure = (self.p.remain_th - remain_c_val) / self.p.remain_th * self.p.max_cl_pressure

        if cur_score + cl_pressure < self.p.change_lane_th:
            msg['state'] = State.acc
            msg['txt'] = 'Threat detected during pre-lane-changing! (score: {}, pressure: {}), fallback to ACC'\
                .format(cur_score, cl_pressure)
            msg['target_lane'] = None
            return msg

        State.signaling_turn_light(direction, self.p.light_freq)
        if self.start_time < 0:
            self.start_time = time_stamp
            return msg
        else:
            if time_stamp > self.start_time + self.p.turn_signaling_iter_num:
                if direction == 'l':
                    msg['state'] = State.l_turn
                else:
                    msg['state'] = State.r_turn
                msg['txt'] = 'Switch from pre-turn signaling to lane changing.'
                return msg
        return msg


class StateLPreTurn(StatePreTurn):
    def update(self, loc_hist, perc, msg, time_stamp):
        return super(StatePreTurn, self).update(self, loc_hist, perc, msg, time_stamp)


class StateRPreTurn(StatePreTurn):
    def update(self, loc_hist, perc, msg, time_stamp):
        return super(StatePreTurn, self).update(self, loc_hist, perc, msg, time_stamp)
