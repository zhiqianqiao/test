import numpy as np
__author__ = 'xhou'


class Traj:
    def __init__(self, keypts=[]):
        self.feats = dict()
        self.keypts = keypts
        self.ptnum = len(keypts)
        self.collision_score = np.nan  # state independent
        self.physics_score = np.nan  # state independent
        self.human_score = np.nan  # state dependent

    def collision_check(self):
        """
        getting a collision score that measures the risk of collision
        :return:
        """
        max_risk = collision_risk(coliision_information_table)