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

    def __init__(self, nav_map, perc_parser, p):
        self.nav_map = nav_map
        self.perc_parser = perc_parser
        self.p = p
        self.target_dist = self.p.safe_distance
        self.target_speed = 30

    def update(self, loc_hist, perc, msg):
        pass

    @staticmethod
    def signaling_turn_light(direction, freq):
        # 'l' or 'r' for left, and right light
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

