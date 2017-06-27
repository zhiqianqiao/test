__author__ = 'xhou'

class States:
    def __init__(self):
        self.loc_hist = deque([], maxlen=p.hist_len)
        pass