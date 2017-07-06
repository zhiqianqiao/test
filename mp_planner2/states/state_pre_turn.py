from .state_base import State
__author__ = 'xhou'


class StatePreTurn(State):
    def update_state(self, v_info, perc, in_memory):
        loc = v_info['abs_loc']
        timestamp = v_info['timestamp']
        turn_info = in_memory['turn_info']
        planned_state = State.l_turn if turn_info['direction'] == 'l' else State.r_turn
        prev_state = State.l_pre_turn if turn_info['direction'] == 'l' else State.r_pre_turn
        State.signaling_turn_light(planned_state, self.p.light_freq)

        self.perc_parser.parse(v_info, perc)

        state_list = [State.defense, planned_state]
        bottomup_scores = dict()
        topdown_scores = dict()
        safety_log = dict()

        for state in state_list:
            bottomup_scores[state], safety_log[state] = self.perc_parser.safety_check(State.acc, state)

        if cur_score < self.p.change_lane_interrupt_th:
            debug_msg = 'Lane changing aborted! Fallback to defense'
            in_memory.update_memory(prev_state, State.defense, debug_msg, timestamp, v_info=v_info, perc_info=perc)
            return in_memory

        virtual_dist, virtual_speed = self.perc_parser.get_front_vehicle(v_info, planned_state)
        traj, _ = self.traj_gen.generate(virtual_dist, virtual_speed, v_info, planned_state)
        if turn_info['start_time'] + self.p.turn_signaling_iter_num < timestamp:
            debug_msg = 'Go for lane changing.'
            in_memory.update_memory(prev_state, planned_state, debug_msg, timestamp, traj=traj)
            return in_memory
        else:
            if self.nav_map.remaining(loc) < self.p.change_lane_th:
                debug_msg = 'Not enough remaining lane to perform lane changing. Detour!'
                in_memory.update_memory(prev_state, State.detour, debug_msg, timestamp)
                return in_memory
            debug_msg = 'Flash n wait for lane changing'



        if timestamp > self.start_time + self.p.turn_signaling_iter_num:
            self.start_time = -1
            in_memory['txt'] = 'Switch from pre-turn signaling to lane changing.'
            in_memory['state'] = planned_state
            virtual_dist, virtual_speed = self.perc_parser.get_front_vehicle(v_info, planned_state)
            in_memory['traj'], _ = self.traj_gen.generate(virtual_dist, virtual_speed, v_info, planned_state)
            return in_memory
        else:
            in_memory['txt'] = 'Pre-lane chaning in progress...... still waiting......'
            virtual_dist, virtual_speed = self.perc_parser.get_front_vehicle(v_info, State.acc)
            in_memory['traj'], _ = self.traj_gen.generate(virtual_dist, virtual_speed, v_info, State.acc)
            return in_memory


class StateLPreTurn(StatePreTurn):
    def update(self, vehicle_info, perc, msg):
        return super(StatePreTurn, self).update(self, vehicle_info, perc, msg)


class StateRPreTurn(StatePreTurn):
    def update(self, vehicle_info, perc, msg):
        return super(StatePreTurn, self).update(self, vehicle_info, perc, msg)
