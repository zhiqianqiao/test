from .state_base import State
__author__ = 'xhou'


class StateDefense(State):
    def update_state(self, v_info, perc, in_memory):
        # TODO: not implemented yet
        self.perc_parser.parse(v_info, perc, in_memory)
        virtual_dist, virtual_speed = self.perc_parser.get_front_vehicle(v_info, State.defense)
        self.msg['traj'], is_finished = self.traj_gen.generate(virtual_dist, virtual_speed, v_info, 'emergency')
        if is_finished:
            self.msg['state'] = State.acc
            self.msg['target_lane'] = None
            self.msg['txt'] = 'Lane change aborted.......'
        else:
            self.msg['txt'] = 'In DEFENSE mode, merging into nearest lane'
        return self.msg
