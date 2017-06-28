# Created by Yufei Zhao on Jun. 25, 2017

import debugging_tools.octopus_logger as oclog
import math
from tsmap import *
from data_structures.pose import Pose

class NPCPerception:

    def __init__(self, map_file_path):
        self.map_info = HDMap(map_file_path)
        if self.map_info.lane_size() == 0:
            oclog.warn('A map must be given. Input does not have lanes.')
            raise Exception('A map must be given. Input does not have lanes.')


    def info2perception(self, info):



class NPCFSM:

    def __init__(self):


MERGE_LEFT = 'merge_left'
MERGE_RIGHT = 'merge_right'
ACC = 'acc'
EMERGENCY = 'emergency'

# to handle:
# speed_list too short
# speed_list horizon too short (can finish merge)
# merge distance too short (idx2 + 1 out of index)
# position outside of lane
# multiple candidate lanes
# trajectory status keeping
# return trajectory keeping status
# distance too short (actual distance less than our distance)
# assume no successive merging commands
# assume the road won't have large heading change during merging






class TrajGenerator(object):

    def __init__(self, tsmap_handler):
        self.traj_handler = TrajMapHandler(tsmap_handler)
        self._last_action = None
        self._kp1 = None
        self._kp2 = None
        # merging_phase:
        # -1: not in process
        # 0: merging started
        # 1: between kp1 and kp2
        # 2: > kp2
        self._merging_phase = -1
        self._merge_dist = None
        # the minimum forward distance before merging
        self.min_forward = 2
        self.vx = 0.5
        self.adj_lim = 0.3
        self.change_x_speed = self.traj_handler.lane_width / 4

    def generate(self, action, speed_list, ego_pose, sample_number, aux):
        # speed_list is a list of speed on road direction
        dvat, t_interval = self._generate_dvat(speed_list, sample_number, ego_pose.speed)
        if action == self._last_action:
            status = self._check_status(ego_pose)
            if status == 0:
                return self._generate_merging_traj(dvat, t_interval, ego_pose, aux)
        return self._generate_traj(dvat, t_interval, action, ego_pose, aux)

    def _check_status(self, ego_pose):
        x, y = ego_pose.position
        if self._status == 0:
            rx, ry, heading = self._kp1
            direction = math.atan2(ry - y, rx - x)
            if abs(direction - heading) < math.pi:
                return 0
            else:
                return 1
        elif self._status == 1:
            rx, ry, heading = self._kp2
            direction = math.atan2(ry - y, rx - x)
            if abs(direction - heading) < math.pi:
                return 1
            else:
                return 2
        else:
            return self._status




    def _generate_merging_traj(self, dvat, t_interval, ego_pose, aux):
        x, y = ego_pose.position
        lane, l, d = self.traj_handler.get_lane_and_coord(x, y)
        lane1, l1, d1 = self.traj_handler.get_lane_and_coord(self._kp1[0], self._kp1[1])
        lane2, l2, d2 = self.traj_handler.get_lane_and_coord(self._kp2[0], self._kp2[1])
        idx = 0
        dist = 0
        clane = lane
        cl = l
        while clane != lane1:
            dist += self.traj_handler.get_lane_length(clane) - cl
            cl = 0
            clane = clane.front_lanes[aux[idx]]
            idx += 1
        dist += l1
        idx1 = self._get_turn_point_index(dist, dvat)
        idx2 = self._get_turn_point_index(dist + self._merge_dist, dvat)
        laterals1, centrals1 = self._generate_laterals(dvat[:idx1 + 1], t_interval, d, self._last_action)
        points1, aux = self.traj_handler.forward2coord(lane, l, centrals1, laterals1, aux)
        trans_points = self._connect(self._kp1, self._kp2, dvat[idx1: idx2])
        centrals2 = [dvat[i][0] for i in xrange(idx2, len(dvat))]
        laterals2 = [0 for i in xrange(idx2, len(dvat))]
        points2, aux = self.traj_handler.forward2coord(lane2, l2 - dvat[idx2][0], centrals2, laterals2, aux)
        del points1[-1]
        del points2[0]
        del dvat[idx2]
        del dvat[idx1]
        return self._generate_rslts(points1 + trans_points + points2, dvat)


    def _generate_traj(self, dvat, t_interval, action, ego_pose, aux):
        x, y = ego_pose.position
        lane, l, d = self.traj_handler.get_lane_and_coord(x, y)
        if action == ACC:
            laterals, centrals = self._generate_laterals(dvat, t_interval, d, action)
            points, _ = self.traj_handler.forward2coord(lane, l, centrals, laterals, aux)
            rslts = self._generate_rslts(points, dvat)
            return rslts
        elif action == MERGE_LEFT or action == MERGE_RIGHT:
            idx1 = self._get_turn_point_index(self.min_forward, dvat)
            if idx1 == -1:
                print "speed profile too short"
                return False

            laterals1, centrals1 = self._generate_laterals(dvat[:idx1 + 1], d, action)
            points1, aux = self.traj_handler.forward2coord(lane, l, centrals1, laterals1, aux)
            turn_position1 = points1[-1]
            self._kp1 = turn_position1
            if action == MERGE_LEFT:
                lateral_distance = self.traj_handler.lane_width + laterals1[-1]
            else:
                lateral_distance = self.traj_handler.lane_width - laterals1[-1]
            central_distance = self._get_merge_distance(lateral_distance, dvat[idx1][1])
            idx2 = self._get_turn_point_index(centrals1[-1] + central_distance, dvat)
            if idx2 == -1:
                print "speed profile too short"
                return False
            central_distance = dvat[idx2][0] - centrals1[-1]
            self._merge_dist = central_distance
            old_lane = lane
            lane, l, aux = self.traj_handler.get_sidelane_and_l(turn_position1[0], turn_position1[1],
                                                                action, central_distance, aux)
            turn_position2 = self.traj_handler.get_point_by_lane_coord(lane, l, 0)
            self._kp2 = turn_position2
            trans_points = self._connect(turn_position1, turn_position2, dvat[idx1: idx2])
            centrals2 = [dvat[i][0] for i in xrange(idx2, len(dvat))]
            laterals2 = [0 for i in xrange(idx2, len(dvat))]
            points2, aux = self.traj_handler.forward2coord(lane, l - dvat[idx2][0], centrals2, laterals2, aux)
            del points1[-1]
            del points2[0]
            del dvat[idx2]
            del dvat[idx1]
            self._merging_phase = 0 if old_lane != lane else 0.5
            rslts = self._generate_rslts(points1 + trans_points + points2, dvat)
            return rslts
        elif action == EMERGENCY:
            pass
        else:
            print "Unsupported action"
            return False

    def _generate_rslts(self, points, dvat):
        rslts = []
        for index in xrange(len(points)):
            x, y, heading = points[index]
            rslts.append({'pose': {'acceleration': dvat[index][2], 'jerk': 0, 'curvature': 0,
                                   'position': [x, y], 'speed': dvat[index][1], 'heading': heading},
                          'time': dvat[index][3]})
        return rslts


    def _connect(self, point1, point2, dvat):
        # point1, point2 exclusive
        x1, y1, _ = point1
        x2, y2, _ = point2
        heading = math.atan2(y2 - y1, x2 - x1)
        delta_x = math.cos(heading)
        delta_y = math.sin(heading)
        points = []
        base_d = dvat[0][0]
        for index in xrange(1, len(dvat)):
            dist = dvat[index][0] - base_d
            points.append((x1 + dist * delta_x, y1 + dist * delta_y, heading))
        return points


    def _get_turn_point_index(self, distance, dvat):
        for index in xrange(len(dvat)):
            if dvat[index][0] >= distance:
                return index
        return -1

    def _get_merge_distance(self, lateral_distance, y_speed):
        return y_speed * (lateral_distance / self.change_x_speed)


    def _generate_laterals(self, dvat, t_interval, d, action):
        laterals = []
        centrals = []
        dx = self.vx * t_interval
        if action == ACC:
            for ele in dvat:
                centrals.append(ele[0])
                if d > dx:
                    d -=dx
                elif d < -dx:
                    d += dx
                else:
                    d = 0
                laterals.append(d)
        elif action == MERGE_LEFT:
            for ele in dvat:
                centrals.append(ele[0])
                if d > 0:
                    d -= dx
                    if d < -self.adj_lim:
                        d = -self.adj_lim
                laterals.append(d)
        elif action == MERGE_RIGHT:
            for ele in dvat:
                centrals.append(ele[0])
                if d < 0:
                    d += dx
                    if d > self.adj_lim:
                        d = self.adj_lim
                laterals.append(d)
        else:
            print "Unsupported action in laterals"
            return None, None
        return laterals, centrals


    def _generate_dvat(self, speed_list, sample_number, c_v):
        v_end, t_end = speed_list[-1]
        t_interval = t_end / sample_number
        # d, d', d'', t
        dvat = []
        n_v, n_t = speed_list[0]
        c_a = (n_v - c_v) / n_t
        t_past = 0
        c_index = 0
        delta_v = c_a * t_interval
        delta_dd = 0.5 * delta_v * t_interval
        dist = 0
        # sample_number - 1 to avoid float precision problem
        for i in range(sample_number - 1):
            t_past += t_interval
            if t_past < n_t:
                dist += c_v * t_interval + delta_dd
                c_v += delta_v
                dvat.append([dist, c_v, c_a, t_past])
            else:
                c_index += 1
                n_v, n_t = speed_list[c_index]
                c_a = (n_v - c_v) / (n_t - t_past + t_interval)
                delta_v = c_a * t_interval
                delta_dd = 0.5 * delta_v * t_interval
                dist += c_v * t_interval + delta_dd
                c_v += delta_v
                dvat.append([dist, c_v, c_a, t_past])
        dvat.append([dist + (v_end + c_v) * (t_end - t_past) / 2,
                     v_end, (v_end - c_v) / (t_end - t_past), t_end])
        return dvat, t_interval






