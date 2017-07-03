from tsmap import TSMap
from map.nav_sub_map import NavMap
from perc_parser import PercParser
from predictor import Predictor
from src.states.state_acc import StateACC
from src.states.state_base import *
from src.states.state_turn import StateLTurn, StateRTurn
from src.traj.traj_generator import TrajGenerator

__author__ = 'xhou'


class Planner:
    def __init__(self, p):
        self.p = p
        self.nav_map = NavMap()
        self.traj_gen = TrajGenerator()
        self.perc_parser = PercParser(self.nav_map, p)
        self.loc_hist = []
        self.timestamp = 0

        self.acc = StateACC(self.nav_map, self.perc_parser, self.traj_gen, p)
        self.l_pre_turn = StateLTurn(self.nav_map, self.perc_parser, self.traj_gen, p)
        self.r_pre_turn = StateRTurn(self.nav_map, self.perc_parser, self.traj_gen, p)
        self.l_turn = StateLTurn(self.nav_map, self.perc_parser, self.traj_gen, p)
        self.r_turn = StateRTurn(self.nav_map, self.perc_parser, self.traj_gen, p)
        self.emergency = StateEmergency(self.nav_map, self.perc_parser, self.traj_gen, p)
        self.detour = StateDetour(self.nav_map, self.perc_parser, self.traj_gen, p)
        self.term = StateTerm(self.nav_map, self.perc_parser, self.traj_gen, p)

        self.state = self.acc
        self.predictor = Predictor(frame_to_pred=5, reg_win=20)

    def update(self, loc, ego_v, raw_perc, msg):
        self.loc_hist.append(loc)
        if len(self.loc_hist) > self.p.loc_hist_len:
            self.loc_hist = self.loc_hist[-self.p.loc_hist_len]
        self.timestamp += 1

        vehicle_info = dict()
        vehicle_info['loc_hist'] = self.loc_hist
        vehicle_info['ego_v'] = ego_v
        vehicle_info['timestamp'] = self.timestamp

        perc = self.predictor.update(raw_perc, self.timestamp)

        msg = self.state.update(vehicle_info, perc, msg)

        self.state = getattr(self, msg['state'])
        self.state.gen_traj()

        return msg
