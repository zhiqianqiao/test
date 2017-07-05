__author__ = 'xhou'


class State(object):
    l_turn = 'l_turn'
    acc = 'acc'
    r_turn = 'r_turn'
    l_pre_turn = 'l_pre_turn'
    r_pre_turn = 'r_pre_turn'
    defense = 'defense'
    term = 'term'
    emergency = 'emergency'
    detour = 'detour'
    valid_states = {l_turn, r_turn, l_pre_turn, r_pre_turn, acc, defense}

    def __init__(self, perc_parser, traj_gen, p):
        self.nav_map = None
        self.perc_parser = perc_parser
        self.traj_gen = traj_gen
        self.p = p
        self.start_time = -1
        self.msg = {'state': None, 'target_lane': None, 'txt': '',
                    'scores': {State.l_turn: 0.0, State.r_turn: 0.0, State.acc: 0.0}}

    def update_map(self, nav_map):
        self.nav_map = nav_map
        
    def update_state(self, v_info, perc, msg):
        pass

    @staticmethod
    def signaling_turn_light(state, freq):
        # state in {State.l_turn, State.r_turn}
        pass


class StateException(State):
    def update_generic(self, error_msg):
        pass


class StateEmergency(StateException):
    pass


class StateDetour(StateException):
    pass


class StateTerm(StateException):
    pass

