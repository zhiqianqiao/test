from nav_map import NavMap
from src.states.state_base import *
from src.states.state_acc import StateACC
from src.states.state_turn import StateLTurn, StateRTurn


__author__ = 'xhou'


class Planner:
    def __init__(self, p):
        self.p = p
        self.nav_map = NavMap()
        self.loc_hist = []

        self.acc = StateACC(self.nav_map, p)
        self.l_turn = StateLTurn(self.nav_map, p)
        self.r_turn = StateRTurn(self.nav_map, p)
        self.emergency = StateEmergency(self.nav_map, p)
        self.detour = StateDetour(self.nav_map, p)
        self.term = StateTerm(self.nav_map, p)

        self.state = self.acc

    def update(self, loc, in_perc, msg):
        self.loc_hist.append(loc)
        if len(self.loc_hist) > self.p.loc_hist_len:
            self.loc_hist = self.loc_hist[-self.p.loc_hist_len]
        new_state, msg = self.state.update(loc, in_perc, msg)
        self.state = getattr(self, new_state)
        self.state.gen_traj()

        return msg
