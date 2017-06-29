from nav_map import NavMap
from intention import Intention
from traj_mgr import TrajMgr, Traj
from parser import LaneParser, CarParser

__author__ = 'xhou'


class StateMachine:
    def __init__(self, p):
        self.p = p
        self.state = Intention.acc

        self.nav_map = NavMap()
        self.acc_intention = Intention(nav_map=self.nav_map, p=self.p)
        self.traj_mgr = TrajMgr()
        self.loc_hist = []
        self.change_lane_target = []

    def update(self, loc, in_perc):
        self.loc_hist.append(loc)
        if len(self.loc_hist) > self.p.loc_hist_len:
            self.loc_hist = self.loc_hist[-self.p.loc_hist_len]
        self.lane_parser.parse(self.loc_hist)

        if self.state == Intention.acc:
            cur_intent, intent_msg = self.acc_intention.update_acc(loc=loc, in_perc=in_perc)
            if cur_intent in {Intention.detour, Intention.term, Intention.emergency}:
                raise RuntimeError(intent_msg)

            if cur_intent in {Intention.l_turn, Intention.r_turn}:
                turn_val = 'l' if cur_intent == Intention.l_turn else 'r'
                self.change_lane_target = self.nav_map.get_extension(self.nav_map.get_par_loc(loc, turn_val))

            self.state = cur_intent
            self.traj_mgr.gen_traj(cur_intent)

        if self.state in {Intention.l_turn, Intention.r_turn}:
            is_finished, intent_msg = self.acc_intention.update_change_lane()


