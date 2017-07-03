from tsmap import *
import math


class TrajMap(object):


    def __init__(self):
        self.handler = None
        self.heading_dist = None

    def update(self, tsmap_handler):
        self.handler = tsmap_handler
        self.heading_dist = 1.2 * tsmap_handler.REF_PT_GAP

    @property
    def lane_width(self):
        return self.handler.LANE_WIDTH

    def _get_heading(self, lane, ref_pt):
        temp_p, dist = lane.move_forward(ref_pt, self.heading_dist)
        if dist > 0.01:
            delta_p = temp_p - ref_pt
        else:
            temp_p, dist = lane.move_backward(ref_pt, self.heading_dist)
            delta_p = ref_pt - temp_p
        heading = math.atan2(delta_p.y, delta_p.x)
        return heading

    def navigate_by_centrals(self, x, y, centrals, ls, drct, thresh=0):
        """
        navigate from (x, y), along the lane, and sample according to centrals
        :param x:
        :param y:
        :param centrals: list of sample point dist
        :param ls: maximum lateral shift, always >= 0
        :param drct: lateral direction, -1: left; 0: mid; 1: right
        :param thresh: by how much we can pass the mid, always >= 0
        :return: a list of samples, (x, y, heading)
        """
        # TODO: if such precision necessary?
        # if l is defined by waypoints, this will be unnecessary
        lane, l, d = self._get_lane_coord(x, y)
        lane_length = lane.get_dist_to_end(lane.start_pt)
        base_dist = -l
        points = []
        for central in centrals:
            if drct == -1 and d > 0:
                d -= ls
                if d < -thresh:
                    d = -thresh
            elif drct == 0:
                if d > ls:
                    d -= ls
                elif d < -ls:
                    d += ls
                else:
                    d = 0.0
            elif drct == 1 and d < 0:
                d += ls
                if d > thresh:
                    d = thresh
            lateral = d
            adaptive_dist = central - base_dist
            while adaptive_dist > lane_length:
                # TODO: what if there are no front lanes
                lane = lane.front_lanes[0]
                base_dist += lane_length
                adaptive_dist = central - base_dist
                lane_length = lane.get_dist_to_end(lane.start_pt)
            points.append(self._get_point_by_lane_coord(lane, adaptive_dist, lateral))
        return points, d

    def _get_lane_coord(self, x, y):
        p = Point3d(x, y, 0)
        lane = self.handler.get_lane(p)
        ref_p = lane.get_ref_pt(p)
        lane_heading = self._get_heading(lane, ref_p)
        dy = y - ref_p.y
        dx = x - ref_p.x
        my_heading = math.atan2(y - ref_p.y, x - ref_p.x)
        d = math.sqrt(dy ** 2 + dx ** 2)
        heading_delta = lane_heading - my_heading
        l = lane.get_dist_from_start(ref_p)
        l += d * math.cos(heading_delta)
        # if d < 0, on the left; if d > 0, on the right
        d *= math.sin(heading_delta)
        return lane, l, d

    def _get_point_by_lane_coord(self, lane, l, d):
        # start_point = lane.ref_pts[0]
        start_point = lane.start_pt
        ref_p, dist = lane.move_forward(start_point, l)
        delta = l - dist
        heading = self._get_heading(lane, ref_p)
        s = math.sin(heading)
        c = math.cos(heading)
        # we may fix heading to higher precision (by interpolation)
        # now we keep it this way, for simplicity
        return [ref_p.x + delta * c + d * s, ref_p.y + delta * s - d * c, heading]

    def get_correspondent_point(self, x, y, side, forward=0):
        """
        get the corresponding (x', y') from the sibling lane, then forwarding if necessary
        :param x:
        :param y:
        :param side: side < 0: left; side == 0: self; side > 0: right
        :param forward: how much we forward
        :return:
        """
        lane, l, d = self._get_lane_coord(x, y)
        if side < 0:
            sidelane = lane.left_lane
        elif side == 0:
            sidelane = lane
        else:
            sidelane = lane.right_lane
        l += forward
        lane_len = sidelane.get_dist_to_end(sidelane.start_pt)
        while l > lane_len:
            sidelane = sidelane.frontlanes[0]
            l -= lane_len
            lane_len = sidelane.get_dist_to_end(sidelane.start_pt)
        return self._get_point_by_lane_coord(sidelane, l, 0)

    # def _test(self, lane):
    #     return lane.start_pt

