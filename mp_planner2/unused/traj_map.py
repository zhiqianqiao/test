import math
from tsmap import *

__author__ = 'yfzhao'

MERGE_RIGHT = 'MERGE_RIGHT'
MERGE_LEFT  = 'MERGE_LEFT'

class TrajMap(object):

    def __init__(self, tsmap_handler):
        self.handler = tsmap_handler

    @staticmethod
    def _euclidean_dist(p1, p2):
        return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2 + (p1.z - p2.z) ** 2)

    @property
    def lane_width(self):
        return self.handler.LANE_WIDTH

    @staticmethod
    def get_heading(lane, ref_p):
        temp_p, dist = lane.move_forward(ref_p, 0.2)
        if dist < 0.01:
            temp_p, dist = lane.move_backward(ref_p, 0.2)
            tmp = ref_p
            ref_p = temp_p
            temp_p = tmp
        delta_p = temp_p - ref_p
        heading = math.atan2(delta_p.y, delta_p.x)
        return heading

    def get_lane_and_coord(self, x, y, z=0):
        p = Point3D(x, y, z)
        lane = self.handler.get_lane(p)
        ref_p = lane.get_ref_pt(p)
        heading = TrajMap.get_heading(lane, ref_p)
        delta_p = p - ref_p
        my_heading = math.atan2(delta_p.y, delta_p.x)
        heading_delta = heading - my_heading
        d = TrajMap._euclidean_dist(p, ref_p)
        l = lane.get_dist_from_start(ref_p)
        l += d * math.cos(heading_delta)
        d = d * math.sin(heading_delta)
        # if d < 0, on the left; if d > 0, on the right
        return lane, l, d


    def lane_numbering(self,lane):
        right = lane.right_lane
        right_list = []
        while right != None:
            right_list.append(right)
            right = right.right_lane

        left = lane.left_lane
        left_list = []
        while left != None:
            left_list.append(left)
            left = left.left_lane

        left_list = left_list[::-1]
        # self.left_lanes = left_list
        # self.right_lanes = right_list

        return left_list,right_list

    @staticmethod
    def get_lane_length(lane):
        return lane.get_dist_to_end(lane.ref_pts[0])

    @staticmethod
    def get_point_by_lane_coord(lane, l, d):
        start_point = lane.ref_pts[0]
        ref_p, dist = lane.move_forward(start_point, l)
        delta = l - dist
        heading = TrajMap.get_heading(lane, ref_p)
        s = math.sin(heading)
        c = math.cos(heading)
        # we may fix heading to higher precision (by interpolation)
        # now we keep it this way, for simplicity
        return ref_p.x + delta * c + d * s, ref_p.y + delta * s - d * c, heading

    def forward2coord(self, lane, offset, centrals, laterals, aux):
        points = []
        base_dist = -offset
        lane_length = TrajMap.get_lane_length(lane)
        aux_idx = 0
        for index in range(len(centrals)):
            central = centrals[index]
            lateral = laterals[index]
            adaptive_dist = central - base_dist
            if adaptive_dist > lane_length:
                lane = lane.front_lanes[aux[aux_idx]]
                aux_idx += 1
                base_dist += lane_length
                adaptive_dist = central - base_dist
                lane_length = TrajMap.get_lane_length(lane)
            points.append(self.get_point_by_lane_coord(lane, adaptive_dist, lateral))
        return points, aux[aux_idx:]

    def get_sidelane_and_l(self, x, y, action, l, aux):
        clane, cl, cd = self.get_lane_and_coord(x, y)
        idx = 0
        if action == MERGE_RIGHT:
            sidelane = clane.right_lane
        elif action == MERGE_LEFT:
            sidelane = clane.left_lane
        l += cl
        lane_len = self.get_lane_length(sidelane)
        while l > lane_len:
            sidelane = sidelane.frontlanes[aux[idx]]
            idx += 1
            l -= lane_len
            lane_len = self.get_lane_length(sidelane)
        return sidelane, l, aux