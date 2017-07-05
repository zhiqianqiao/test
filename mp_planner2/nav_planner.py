from traj.traj_generator import TrajGenerator
from perc_parser import PercParser
from predictor import Predictor
from states.memory import Memory

from states.state_acc import StateACC
from states.state_pre_turn import StateLPreTurn, StateRPreTurn
from states.state_turn import StateLTurn, StateRTurn
from states.state_defense import StateDefense
from states.state_base import StateEmergency, StateDetour, StateTerm

__author__ = 'xhou'


class Planner:
    def __init__(self, p):
        self.nav_map = None
        self.traj_gen = TrajGenerator()
        self.perc_parser = PercParser(p)
        self.predictor = Predictor(frame_to_pred=5, reg_win=20)
        self.memory = Memory()

        self.acc = StateACC(self.perc_parser, self.traj_gen, p)
        self.l_pre_turn = StateLPreTurn(self.perc_parser, self.traj_gen, p)
        self.r_pre_turn = StateRPreTurn(self.perc_parser, self.traj_gen, p)
        self.l_turn = StateLTurn(self.perc_parser, self.traj_gen, p)
        self.r_turn = StateRTurn(self.perc_parser, self.traj_gen, p)
        self.defense = StateDefense(self.perc_parser, self.traj_gen, p)

        self.emergency = StateEmergency(self.perc_parser, self.traj_gen, p)
        self.detour = StateDetour(self.perc_parser, self.traj_gen, p)
        self.term = StateTerm(self.perc_parser, self.traj_gen, p)

        self.p = p
        self.loc_hist = []
        self.timestamp = 0
        self.state = self.acc

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

        perc = self.predictor.update_predictor(v_info, raw_perc, self.timestamp)
        self.memory = self.state.update_state(v_info, perc, self.memory)
        self.state = getattr(self, self.memory.next_state)
        return self.memory
