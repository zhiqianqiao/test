import numpy as np

__author__ = 'xhou'


class TrajMgr:
    def __init__(self, params):
        self.time_horizon = params.time_horizon
        self.time_interval = params.plan_interval
        self.predictor = Predictor(self.time_horizon, self.time_interval)
        self.prediction = []
        self.traj_list = []
        self.map_data = []
        self.cur_pose = []
        self.prev_traj = []

    def init_context(self, cur_pose, map_data, perception, prev_traj):
        self.traj_list = []
        self.prediction = self.predictor.predict(perception)
        self.map_data = map_data
        self.cur_pose = cur_pose
        self.prev_traj = prev_traj

    def get_feats(self):
        """
        # state independent
        offset
        speed
        accel
        jerk
        curvature
        lateral_jerk
        near_dist

        # state dependent
        heading
        lateral_offset
        out_of_lane (?)

        """
        pass

    def get_scores(self):