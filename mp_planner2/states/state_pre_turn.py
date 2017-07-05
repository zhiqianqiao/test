from .state_base import State
__author__ = 'xhou'


class StatePreTurn(State):
    def update_state(self, v_info, perc, msg):
        loc = v_info['abs_loc']
        timestamp = v_info['timestamp']
        if self.start_time < 0:
            self.start_time = timestamp
        cur_state = msg['state']
        planned_state = State.l_turn if cur_state == State.l_pre_turn else State.r_turn
        self.perc_parser.parse(v_info, perc)
        cur_score = self.perc_parser.safety_check(State.acc, planned_state)

        remain_c_val = self.nav_map.remaining(loc)
        cl_pressure = (self.p.remain_th - remain_c_val) / self.p.remain_th * self.p.max_cl_pressure

        if cur_score + cl_pressure < self.p.change_lane_th:
            self.start_time = -1
            msg['txt'] = 'Threat detected during pre-lane-changing! (score: {}, pressure: {}), fallback to ACC'\
                .format(cur_score, cl_pressure)
            msg['state'] = State.acc
            msg['target_lane'] = None
            virtual_dist, virtual_speed = self.perc_parser.get_front_vehicle(v_info, State.acc)
            msg['traj'], _ = self.traj_gen.generate(virtual_dist, virtual_speed, v_info, State.acc)
            return msg

        State.signaling_turn_light(planned_state, self.p.light_freq)

        if timestamp > self.start_time + self.p.turn_signaling_iter_num:
            self.start_time = -1
            msg['txt'] = 'Switch from pre-turn signaling to lane changing.'
            msg['state'] = planned_state
            virtual_dist, virtual_speed = self.perc_parser.get_front_vehicle(v_info, planned_state)
            msg['traj'], _ = self.traj_gen.generate(virtual_dist, virtual_speed, v_info, planned_state)
            return msg
        else:
            msg['txt'] = 'Pre-lane chaning in progress...... still waiting......'
            virtual_dist, virtual_speed = self.perc_parser.get_front_vehicle(v_info, State.acc)
            msg['traj'], _ = self.traj_gen.generate(virtual_dist, virtual_speed, v_info, State.acc)
            return msg


class StateLPreTurn(StatePreTurn):
    def update(self, vehicle_info, perc, msg):
        return super(StatePreTurn, self).update(self, vehicle_info, perc, msg)


class StateRPreTurn(StatePreTurn):
    def update(self, vehicle_info, perc, msg):
        return super(StatePreTurn, self).update(self, vehicle_info, perc, msg)
