from nav_map import NavMap
from intention import Intention
from traj_mgr import TrajMgr, Traj
from parser import LaneParser, CarParser

__author__ = 'xhou'


class StateMachine:
    def __init__(self, p):
        self.p = p

        self.nav_map = NavMap()
        self.acc_intention = Intention(nav_map=self.nav_map, p=self.p)
        self.traj_mgr = TrajMgr()
        self.loc_hist = []

        self.intent = Intention.acc

        self.change_lane_target = None
        self.traj = None

    def update(self, loc, in_perc):
        self.loc_hist.append(loc)
        if len(self.loc_hist) > self.p.loc_hist_len:
            self.loc_hist = self.loc_hist[-self.p.loc_hist_len]
        self.lane_parser.parse(self.loc_hist)

        if self.intent == Intention.acc:
            self.intent, intent_msg = self.acc_intention.update_acc(self.loc_hist, in_perc)
            if self.intent in {Intention.detour, Intention.term, Intention.emergency}:
                raise RuntimeError(intent_msg)

            if self.intent in {Intention.l_turn, Intention.r_turn}:
                turn_val = 'l' if self.intent == Intention.l_turn else 'r'
                self.change_lane_target = self.nav_map.get_extension(self.nav_map.get_par_loc(loc, turn_val))
                self.traj = None

        if self.intent in {Intention.l_turn, Intention.r_turn}:
            self.intent, intent_msg = \
                self.acc_intention.update_change_lane(self.loc_hist, in_perc, self.intent, self.change_lane_target)

            if self.intent == Intention.acc:
                self.change_lane_target = None
                self.traj = None

            if self.intent == Intention.defense:
                self.change_lane_target = None
                self.traj = None

        if self.intent == Intention.defense:
            self.intent = Intention.acc
            # TODO: add some defensive measures here

        self.traj = self.traj_mgr.gen_traj(self.intent, self.loc_hist, in_perc, self.traj)
        return


