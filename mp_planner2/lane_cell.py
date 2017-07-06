import numpy as np
from predictor import Predictor
from tsmap import Point3d
__author__ = 'xhou'


class LaneCell:
    nearest_only = {0, 2, 3, 5, 6, 8}

    def __init__(self, p):
        self.nav_map = None
        self.turn_info = None
        self.p = p
        self.cells = [[] for _ in range(p.cell_num)]

    def update_map(self, nav_map):
        self.nav_map = nav_map

    def put_cars(self, perc):
        # TODO: consider car size issue
        assert isinstance(perc, dict), 'perc is not a dictionary'
        cell_pool = [list() for i in range(self.p.cell_num)]
        cell_nearest = [None] * self.p.cell_num
        cell_status = [set() for i in range(self.p.cell_num)]

        for car_id, car in perc.iteritems():
            for cell_itr, cur_lane_ext in enumerate(self.cells):
                car_loc = [car['abs_x'], car['abs_y']]

                # TODO: push Xu Ke to fix this!
                p = Point3d(car_loc[0], car_loc[1], 0)
                if cur_lane_ext.contains_loc(self.nav_map.submap.get_lane(p), p):
                    cell_pool[cell_itr].append(car)
                    break

        for cell_itr in LaneCell.nearest_only:
            n_dist = np.inf
            for car in cell_pool[cell_itr]:
                if car['rel_l'] < n_dist:
                    cell_nearest[cell_itr] = car
                    n_dist = car['rel_l']

        for cell_itr, car_list in enumerate(self.cell_pool):
            if cell_itr in LaneCell.nearest_only:
                car_list = [cell_nearest[cell_itr]]

            for car in car_list:
                if not car:
                    continue
                for state_itr in Predictor.all_states:
                    if car['state'][state_itr]:
                        cell_status[cell_itr].add(state_itr)
        return cell_pool, cell_nearest, cell_status

    def renew_cells(self, v_info, perc, in_memory):
        p = self.p
        speed = max(v_info['speed'], p.min_driving_speed)

        loc = v_info['abs_loc']
        turn_info = in_memory['turn_info']
        direction = turn_info['direction']
        rev_direction = 'l' if direction == 'r' else 'r'
        if turn_info:
            if loc in turn_info['init_lane']:
                turn_info['init_lane'] = self.nav_map.get_extension(self.nav_map.get_par_loc(loc, 'c'))
                turn_info['target_lane'] = self.nav_map.get_extension(self.nav_map.get_par_loc(loc, direction))
            else:
                # TODO: push Xu Ke to fix this!
                p = Point3d(loc[0], loc[1], 0)
                assert turn_info['target_lane'].contains_loc(self.nav_map.submap.get_lane(p), p),\
                    'loc not in either init or target lane segment! Possible NavMap bug'
                turn_info['init_lane'] = self.nav_map.get_extension(self.nav_map.get_par_loc(loc, rev_direction))
                turn_info['target_lane'] = self.nav_map.get_extension(self.nav_map.get_par_loc(loc, 'c'))
                loc = self.nav_map.get_par_loc(loc, direction)

        cell_upper = np.ones(p.cell_num) * p.cell_border_1 * speed
        cell_lower = np.ones(p.cell_num) * p.cell_border_6 * speed

        cell_upper[[1, 7, 9, 10]] = p.cell_border_2 * speed
        cell_upper[4] = p.cell_border_3 * speed
        cell_upper[[2, 5, 8]] = p.cell_border_5 * speed

        cell_lower[[0, 6]] = p.cell_border_2 * speed
        cell_lower[3] = p.cell_border_3 * speed
        cell_lower[[9, 10]] = p.cell_border_4 * speed
        cell_lower[[1, 4, 7]] = p.cell_border_5 * speed

        l_loc = self.nav_map.get_par_loc(loc, 'l')
        r_loc = self.nav_map.get_par_loc(loc, 'r')
        ll_loc = self.nav_map.get_par_loc(loc, 'll')
        rr_loc = self.nav_map.get_par_loc(loc, 'rr')
        loc_list = [l_loc, l_loc, l_loc, loc, loc, loc, r_loc, r_loc, r_loc, ll_loc, rr_loc]
        loc_list = [self._to_list(loc) for loc in loc_list]

        for i in range(self.p.cell_num):
            self.cells[i] = self.nav_map.get_extension(loc_list[i], cell_upper[i], cell_lower[i])

        return self.put_cars(perc)


    @staticmethod
    def _to_list(loc):
        if loc is None or type(loc) is list:
            return loc
        return [loc.x, loc.y]
