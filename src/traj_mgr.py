import numpy as np

__author__ = 'xhou'


class TrajMgr:
    def __init__(self, params, hdmap):
        self.time_horizon = params.time_horizon
        self.time_interval = params.plan_interval
        self.predictor = Predictor(self.time_horizon, self.time_interval)
        self.prediction = []
        self.traj_list = []
        self.map_data = []
        self.cur_pose = []
        self.prev_traj = []
        self.hdmap = hdmap

    def init_context(self, map_data, cur_pose, perception, prev_traj):
        self.traj_list = []
        self.prediction = self.predictor.predict(perception)
        self.map_data = map_data
        self.cur_pose = cur_pose
        self.prev_traj = prev_traj

    def gen_traj(self, intent):
        if intent == Intent.acc:
            propose_trajectories('acc', self.cur_pose, self.prediction, self.prev_traj, self.map_data)

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



def propose_trajectories(mode, cur_pose, predictions, hdmap, prev, traj, params=DEFAULT_PARAMS):
    surrounding_perception = SurroundingPerception(perception_info, hdmap)
    anchors = sample_anchors(perception_info, surrounding_perception,
                             mode, params)
    paths = path_generation(anchors, cur_pose, params,
                            defensive=self.planner_setting.defensive_path_generation)
    speed_profiles = generate_speed_profile(cur_pose, surrounding_perception,
                                            params['speed_profile_generation_mode'], mode, params)
    for path in paths:
        for speed_profile in speed_profiles:
            trajectories.append(BasicPlannerTrajectory(path, speed_profile))

    time_step = params['time_interval']
    traj_list = []
    for traj in trajectories:
        poses = traj.get_pose_at_time_list('')
        traj_list.append(Trajectory(poses, self.time_step, perception_time_stamp))
    return traj_list