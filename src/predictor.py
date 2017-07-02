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
    rel_lv (opt)
    rel_d (opt)
    rel_dv (opt)
    rel_x
    rel_y
    rel_xv
    rel_yv
    state
    '''


    def __init__(self):
        pass

    def update(self, raw_perc):
        return processed_perc

    def add_virtual_car(self, dist, speed):
        pass
