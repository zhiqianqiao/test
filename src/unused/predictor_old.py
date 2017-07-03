__author__ = 'xhou'


class Predictor:
    normal = 'normal'
    overspeed = 'overspeed'
    underspeed = 'underspeed'
    l_turn = 'l_turn'
    r_turn = 'r_turn'
    unknown = 'unknown'
    reckless = 'reckless'

    '''
    For each car:
    rel_l (opt)
    rel_d (opt)
    rel_lv (opt)
    rel_dv (opt)
    rel_x
    rel_y
    rel_xv
    rel_yv
    state
    state_conf
    '''

    def __init__(self):
        pass

    def update(self, raw_perc, time_stamp):
        processed_perc = raw_perc
        return processed_perc
