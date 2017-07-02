__author__ = 'xhou'


class Predictor:
    normal = 'normal'
    accel = 'accel'
    brake = 'brake'
    l_turn = 'l_turn'
    r_turn = 'r_turn'
    unknown = 'unknown'
    reckless = 'reckless'

    def __init__(self):
        pass

    def update(self, in_perc):
        pass

    def predict(self, car_id):
        pass

    def add_virtual_car(self, dist, speed):
        pass
