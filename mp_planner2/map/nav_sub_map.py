from tsmap import TSMap, Lane, Point3d, Bound
from _lane_extension import LaneExtension
from _lane_extension import LaneSeg
from collections import deque



class NavMap:
    def __init__(self):
        self.submap = None
        self.nav_forward = None
        self.nav_backward = None
        self.nav_sibling = None
        self.recom_route = []
        self.cur_extension = None

    def update_navmap(self, ros_data):
        self.nav_forward = {}
        for lst in ros_data.nav_fwd:
            self.nav_forward[lst[0]] = lst[1:]
        self.nav_backward = {}
        for lst in ros_data.nav_bwd:
            self.nav_backward[lst[0]] = lst[1:]

        self.nav_sibling = {}
        for lst in ros_data.nav_sib:
            self.nav_sibling[lst[0]] = lst[1:]
        self.recom_route = ros_data.best_path

    def update_submap(self, handle):
        self.submap = handle

    def print_map(self):
        print self.nav_forward
        print self.nav_backward
        print self.nav_sibling

    def get_par_loc(self, loc, lane_name='l'):
        if self.submap is None:
            print "# ERROR # No submap loaded !"
            return

        tmp = self.submap.get_lane(self.submap.get_ref_pt(loc))
        cnt = len(lane_name)

        if lane_name[0] == 'l':
            for i in range(cnt):
                tmp = tmp.left_lane
                if tmp is None:
                    return None
        elif lane_name[0] == 'r':
            for i in range(cnt):
                tmp = tmp.right_lane
                if tmp is None:
                    return None
        return tmp.get_ref_pt(loc)

    def remaining_distance(self, loc, max_range=5000.0, mode='decision'):
        # Remaining distance of all possible front routes
        if mode == 'all':
            def remain_rec(lanes, res):
                if res > max_range:
                    return [max_range]
                if lanes == [] or lanes is None:
                    return [res]
                r = []
                for l in lanes:
                    temp = remain_rec(
                        l.front_lanes,
                        res +
                        l.get_dist_to_end(
                            l.ref_pts[0]))
                    map(r.append, temp)
                print r
                return [x+res for x in r]
            # End of assist function

            ref = self.submap.get_ref_pt(loc)
            lane = self.submap.get_lane(ref)

            res = lane.get_dist_to_end(ref)
            res = remain_rec(lane.front_lanes, res)

            for i in range(len(res)):
                if res[i] > max_range:
                    res[i] = max_range
            return res

        # Remaining distance to make a decision to prevent hit end or crossing
        elif mode == 'decision':
            def remain_rec(lanes, res):
                if res > max_range:
                    return max_range
                elif lanes == [] or lanes is None or len(lanes) > 1:
                    return res
                else:
                    l = lanes[0]
                    return remain_rec(l.front_lanes, res +
                                      l.get_dist_to_end(l.ref_pts[0]))
            # End of assist function

            ref = self.submap.get_ref_pt(loc)
            lane = self.submap.get_lane(ref)
            res = lane.get_dist_to_end(ref)
            res = remain_rec(lane.front_lanes, res)

            if res > max_range:
                return max_range
            else:
                return res

    def recom(self, loc):
        if self.recom_route == []:
            print "# ERROR# No pre-generated recom route for function <recom>"
            return []

        ref = self.submap.get_ref_pt(loc)
        lane = self.submap.get_lane(ref)
        res = [False, lane.id in self.recom_route, False]
        l = lane.left_lane
        r = lane.right_lane

        while l !=None:
            if l.id in self.recom_route:
                res[0] = True
                break
            l=l.left_lane

        while r !=None:
            if r.id in self.recom_route:
                res[2] = True
                break
            r=r.right_lane

        return res

    def calc_point_id(self, lane, dist, mode='quick'):
        if mode == 'quick':
            # NOTE: Here use an assume that the ref points are distributed
            # perfectly even, its a quick solution, but not highly precise.
            length = lane.get_dist_to_end(lane.ref_pts[0])
            cnt = len(lane.ref_pts)
            id = int(cnt * (dist / length))
            if id > cnt - 1:
                id = cnt - 1
            if id < 0:
                id = 0
            print id
            return id

    def remaining(self, loc, forward_len=300.0):
        return self.get_extension(
            loc, forward_len=forward_len, backward_len=0)

    def get_extension(self, loc, forward_len=300.0, backward_len=300.0):
        ref = self.submap.get_ref_pt(loc)
        lane = self.submap.get_lane(ref)
        lane_id = lane.id

        # Nav-info:
        #  - self.nav_forward
        #  - self.nav_backward
        ext = LaneExtension()

        # Recursion Function
        def forward_search(l, r):
            length = l.get_dist_to_end(l.ref_pts[0])
            if r < length:
                eref = self.calc_point_id(l, r)
                ext.add_seg(l, 0, eref)
            else:
                ext.add_seg(l, 0, -1)
                remain = r - length
                if l.id in self.nav_forward:
                    nav = self.nav_forward[l.id]

                for fl in l.front_lanes:
                    if fl.id not in nav:
                        continue
                    else:
                        forward_search(fl, remain)

        def backward_search(l, r):
            length = l.get_dist_to_end(l.ref_pts[0])
            if r < length:
                sref = self.calc_point_id(l, length - r)
                ext.add_seg(l, sref, -1)
            else:
                ext.add_seg(l, 0, -1)
                remain = r - length
                if l.id in self.nav_backward:
                    nav = self.nav_backward[l.id]

                for bl in l.back_lanes:
                    if bl.id not in nav:
                        continue
                    else:
                        backward_search(bl, remain)

        frd = lane.get_dist_to_end(ref)
        bkd = lane.get_dist_from_start(ref)
        ptid = self.calc_point_id(lane, bkd)

        # Search forward
        if forward_len > 0.0:
            remain = forward_len - frd
            # print remain
            if remain > 0:
                print "!"
                ext.add_seg(lane, ptid, -1)
                for fl in lane.front_lanes:
                    forward_search(fl, remain)
            else:
                ext.add_seg(
                    lane, ptid, self.calc_point_id(
                        lane, bkd + forward_len))

        # Search backward
        if backward_len > 0.0:
            remain = backward_len - bkd
            if remain > 0:
                ext.add_seg(lane, 0, ptid)
                for bl in lane.back_lanes:
                    backward_search(bl, remain)
            else:
                ext.add_seg(
                    lane, self.calc_point_id(
                        lane, bkd - backward_len), ptid)

        self.cur_extension = ext
        return ext

    def in_extension(self, loc_list, lane_extension):
        ret=[]
        for loc in loc_list:
            lan = self.submap.get_lane(loc)
            if lan is None:
                # Point not on any lane
                ret.append(False)
                continue
            if not lane_extension.contains_loc(lan, loc):
                ret.append(False)
                continue
            ret.append(True)
        return ret

    def get_lane_coords(self, loc, x, y):
        """
        Get the lane coordinates of (x, y) relative to loc.
        This function works on whole submap without filtering out lanes that cannot reach des.
        :param loc: the point of reference for the coordinate conversion
        :param x: 
        :param y: 
        :return: (l, d) when (x, y) can be mapped (perpendicularly) to the lane 
        (or its primary front or back lanes). Returns None, when there is no such
        lane that (x, y) can be perpendicularly mapped to (this happens when the
        extension of current lane reaches an end).
        """

        # locate the lane contains the target location (x, y)
        target_pt = Point3d(x, y)
        ref_pt = self.submap.get_ref_pt(target_pt)
        lane = self.submap.get_lane(ref_pt)

        # check target location (x, y) in nav_map:
        if lane.id not in self.nav_sibling.keys:
            return None

        # mark the left/right neighboring
        neighbors_of_target = set()
        neighbors_of_target.add(lane)

        tmp = lane
        while lane.left_lane is not None:
            left = lane.left_lane
            neighbors_of_target.add(left)
            lane = left

        lane = tmp
        while lane.right_lane is not None:
            right = lane.right_lane
            neighbors_of_target.add(right)
            lane = right

        # search forward and backward for the target lane or its neighboring lanes
        target_lane = None
        total_length = None
        visited = set()
        cur_lane = self.submap.get_lane(loc)
        queue = deque([(cur_lane, 0)])
        while len(queue) > 0:
            lane, length = queue.popleft()
            visited.add(lane)
            if lane in neighbors_of_target:
                target_lane = lane
                total_length = length
                break
            front_lane = lane.front_lane
            if front_lane is not None and front_lane not in visited:
                queue.append((front_lane, length + self._get_lane_length(front_lane)))
            back_lane = lane.back_lane
            if back_lane is not None and back_lane not in visited:
                queue.append((back_lane, length - self._get_lane_length(back_lane)))

        # target not in submap
        if target_lane is None:
            return None

        # project the target point onto the target lane
        target_ret_pt = target_lane.get_ref_pt(target_pt)

        if total_length > 0:
            # in the front
            l = total_length - target_lane.get_dist_to_end(target_ret_pt) \
                + cur_lane.get_dist_to_end(loc)
        elif total_length < 0:
            # in the back
            l = total_length + target_lane.get_dist_from_start(target_ret_pt) \
                - cur_lane.get_dist_from_start(loc)
        else:
            # total_length == 0, i.e., loc and target_ret_pt in the same lane segment
            l = cur_lane.get_dist_to_end(loc) - cur_lane.get_dist_to_end(target_ret_pt)

        d = self._euclidean_dis(target_ret_pt, target_pt)
        next_pt, _ = target_lane.move_forward(target_ret_pt, target_lane.ref_pt_gap)

        if self._is_left(target_pt, target_ret_pt, next_pt) < 0:
            d = -d
        return l, d

    def get_virtual_car(self, loc, distance, speed, max_len = 50):
        update_freq = 20.0
        update_dist = speed / update_freq
        dist = distance

        curr_lane = self.submap.get_lane(loc)
        curr_pt = self.submap.get_ref_pt(loc)

        virtual_car_lst = []

        for i in range(max_len):
            next_pt, dist_to_next_p = curr_lane.move_forward(curr_pt, dist)
            remaining_dist = dist - dist_to_next_p
            while remaining_dist > 0:
                next_lane = None
                for front_lane in curr_lane.front_lanes:
                    if front_lane.id in self.nav_forward[curr_lane.id]:
                        next_lane = front_lane
                if next_lane is None:
                    return virtual_car_lst

                curr_lane = next_lane
                curr_pt = next_lane.start_pt
                next_pt, dist_to_next_p = curr_lane.move_forward(curr_pt, dist)
                remaining_dist = dist - dist_to_next_p
            virtual_car_lst.append(next_pt)
            curr_pt = next_pt
            dist = update_dist

        return virtual_car_lst

    @staticmethod
    def _get_lane_length(lane):
        """
        Returns the length of a lane
        :param lane: the lane to calculate its distance
        :return: the length of lane
        """
        return lane.get_dist_to_end(lane.start_pt)

    @staticmethod
    def _euclidean_dis(p1, p2):
        """
        Returns the euclidean distance between p1 and p2
        :param p1: 
        :param p2: 
        :return: the euclidean distance between p1 and p2
        """
        return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)

    @staticmethod
    def _is_left(p, p1, p2):
        """
        tests if a point is Left|On|Right of an line defined by two points.
        :param p: the point to test
        :param p1: first point on the line
        :param p2: second point on the line
        :return: > 0 when p is left of the line through p1 and p2
                == 0 when p is on the line
                 < 0 when p is right of the line
        """
        return (p2.x - p1.x) * (p.y - p1.y) - (p.x - p1.x) * (p2.y - p1.y)
