__author__ = 'xhou'


class NavMap:
    def __init__(self):
        pass

    def get_par_loc(self, loc, lane_name='l'):
        # lane_name in {'c', 'l', 'll', 'r', 'rr'}
        # return lane center point. Return None if lane does not exist
        pass

    def remaining(self, loc):
        # lane_name in {'left', 'right', or 'cur'}
        # if remaining > 3000m return 3000m
        # return in 5ms
        self.get_extension(loc, forward_len=3000, backward_len=0)
        return

    def recom(self, loc):
        # return if {the set of all left lanes}, current lane, {the set of all right lanes} belongs to the recommended node set
        # return in 5ms
        return [True, True, False]

    def get_extension(self, loc, forward_len=300, backward_len=300):
        # get the chain of reachable tile nodes within +300m and -300m that do not require any lane changing
        return LaneExtension()

    def in_extension(self, loc_list, lane_extension):
        for loc in loc_list:
            if not lane_extension.contains_loc(loc):
                return False
        return True

    # TODO: add speed limit
    def get_speed_limit(self, loc):
        pass

    def get_virtual_car(self, loc, distance, speed):
        pass


class LaneExtension:
    def __init__(self):
        pass

    def contains_loc(self, loc):
        pass
