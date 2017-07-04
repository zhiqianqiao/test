from tsmap import TSMap
from perc_parser import PercParser
from predictor import Predictor
from states.state_acc import StateACC
from states.state_base import *
from states.state_turn import StateLTurn, StateRTurn
from traj.traj_generator import TrajGenerator

__author__ = 'xhou'


class Planner:
    def __init__(self, p):
        self.p = p
        self.nav_map = None
        self.traj_gen = TrajGenerator()
        self.perc_parser = PercParser(p)
        self.loc_hist = []
        self.timestamp = 0

        self.acc = StateACC(self.perc_parser, self.traj_gen, p)
        self.l_pre_turn = StateLTurn(self.perc_parser, self.traj_gen, p)
        self.r_pre_turn = StateRTurn(self.perc_parser, self.traj_gen, p)
        self.l_turn = StateLTurn(self.perc_parser, self.traj_gen, p)
        self.r_turn = StateRTurn(self.perc_parser, self.traj_gen, p)
        self.emergency = StateEmergency(self.perc_parser, self.traj_gen, p)
        self.detour = StateDetour(self.perc_parser, self.traj_gen, p)
        self.term = StateTerm(self.perc_parser, self.traj_gen, p)

        self.state = self.acc
        self.predictor = Predictor(frame_to_pred=5, reg_win=20)

    def update_maps(self, ts_map, nav_map):
        self.nav_map = nav_map
        self.traj_gen.update(ts_map)
        
        self.acc.update_map(nav_map)
        self.l_pre_turn.update_map(nav_map)
        self.r_pre_turn.update_map(nav_map)
        self.l_turn.update_map(nav_map)
        self.r_turn.update_map(nav_map)
        self.emergency.update_map(nav_map)
        self.detour.update_map(nav_map)
        self.term.update_map(nav_map)

        self.perc_parser.update_map(nav_map)

    def update(self, v_info, raw_perc, msg):
        v_info['timestamp'] = self.timestamp
        loc = v_info['abs_loc']

        self.loc_hist.append(loc)
        if len(self.loc_hist) > self.p.loc_hist_len:
            self.loc_hist = self.loc_hist[-self.p.loc_hist_len:]
        self.timestamp += 1

        perc = self.predictor.update(v_info, raw_perc, self.timestamp)
        try:
            print v_info['gps_time']
            msg = self.state.update(v_info, perc, msg)
        except:
            print v_info['gps_time']
            a = 12
        self.state = getattr(self, msg['state'])
        return msg
