from src.perc_parser import PercParser
__author__ = 'xhou'


class State:
    l_turn = 'l_turn'
    acc = 'acc'
    r_turn = 'r_turn'
    defense = 'defense'
    term = 'term'
    emergency = 'emergency'
    detour = 'detour'
    valid_states = {l_turn, r_turn, acc, defense}

    def __init__(self, nav_map, p):
        self.nav_map = nav_map
        self.p = p
        self.perc_parser = PercParser(nav_map, p)

    def update(self, loc_hist, in_perc, msg):
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

