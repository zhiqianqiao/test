__author__ = 'xhou'


class Memory:
    def __init__(self):
        self.time_stamp
        self.prev_state = None
        self.next_state = None
        self.debug_msg = ''
        self.scores = dict()
        self.traj = []

    def state_trans(self, src_state, dst_state):
        pass

    def get_traj(self):
        pass