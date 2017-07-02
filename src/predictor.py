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
    rel_l
    rel_lv
    rel_d
    rel_dv
    rel_x
    rel_y
    '''


    def __init__(self):
        pass

    def update(self, in_perc):
        pass

    def add_virtual_car(self, dist, speed):
        pass
