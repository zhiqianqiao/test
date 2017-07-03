from traj_map import TrajMap
import math
from tsmap import *


class State(object):
    valid = {'acc', 'left', 'right', 'undo'}

    def __init__(self):
        pass

    def update(self, action, traj_gen):
        raise NotImplementedError

    @staticmethod
    def _check_state(x, y, kp):
        rx, ry, heading = kp
        direction = math.atan2(ry - y, rx - x)
        if 0.5 * math.pi < abs(direction - heading) < 1.5 * math.pi:
            return 1
        return 0


class Acc(State):

    def __init__(self):
        self.valid = {'acc', 'left', 'right', 'undo'}

    def update(self, action, traj_gen):
        if action in self.valid:
            if action == 'acc' or action == 'undo':
                return 'acc', traj_gen.generate_acc(0)
            if action == 'left':
                return 'm0', traj_gen.generate_merge(action, traj_gen.adj_lim)
            if action == 'right':
                return 'm0', traj_gen.generate_merge(action, traj_gen.adj_lim)
        else:
            raise Exception("in acc, new action: ", action)
            return 'fail', []


class Merge0(State):

    def __init__(self):
        self.valid = {'left', 'right', 'undo'}
        self._action = None

    def update(self, action, traj_gen):
        if self._check_action(action):
            state = self._check_state(traj_gen.ego_x, traj_gen.ego_y, traj_gen._kp1)
            if state == 0:
                if action == 'undo':
                    return 'acc', traj_gen.generate_acc(0)
                return 'm0', traj_gen.follow_phase_0()
            else:
                if action == 'undo':
                    return 'm1', traj_gen.undo_merge()
                return 'm1', traj_gen.follow_phase_1()
        else:
            raise Exception("in merge0, new action: ", action)
            return 'fail', []

    def _check_action(self, action):
        if action not in self.valid:
            return False
        if action == 'undo':
            return True
        return action == self._action

    def set_action(self, action):
        self._action = action


class Merge1(State):

    def __init__(self):
        self.valid = {'left', 'right', 'undo'}
        self._action = None

    def update(self, action, traj_gen):
        if self._check_action(action):
            state = self._check_state(traj_gen.ego_x, traj_gen.ego_y, traj_gen._kp2)
            if state == 0:
                if action != self._action:
                    return 'm1', traj_gen.undo_merge()
                return 'm1', traj_gen.follow_phase_1()
            else:
                return 'acc', traj_gen.generate_acc(0)
        else:
            raise Exception("in merge1, new action: ", action)
            return 'fail', []

    def _check_action(self, action):
        if action not in self.valid:
            return False
        if action == 'undo':
            return True
        return action == self._action

    def set_action(self, action):
        self._action = action

# class MergeLeft0(State):
#     def __init__(self):
#         self.valid = {'left', 'undo'}
#
#     def update(self, action, traj_gen):
#         if action in self.valid:
#             state = self._check_state(traj_gen.ego_pose.position, traj_gen._kp1)
#             if state == 0:
#                 return 'ml0', traj_gen.follow_phase_0()
#             else:
#                 return traj_gen.ml1.update(action, traj_gen)
#         else:
#             return 'fail', []
#
#
# class MergeLeft1(State):
#     def __init__(self):
#         self.valid = {'left', 'undo'}
#
#     def update(self, action, traj_gen):
#         if action in self.valid:
#             state = self._check_state(traj_gen.ego_pose.position, traj_gen._kp2)
#             if state == 0:
#                 return 'ml1', traj_gen.follow_phase_1()
#             else:
#                 return traj_gen.acc.update(action, traj_gen)
#         else:
#             return 'fail', []
#
#
# class MergeRight0(State):
#     def __init__(self):
#         self.valid = {'right', 'undo'}
#
#     def update(self, action, traj_gen):
#         if action in self.valid:
#             state = self._check_state(traj_gen.ego_pose.position, traj_gen._kp1)
#             if state == 0:
#                 return 'mr0', traj_gen.follow_phase_0()
#             else:
#                 return traj_gen.mr1.update(action, traj_gen)
#         else:
#             return 'fail', []
#
#
# class MergeRight1(State):
#
#     def __init__(self):
#         self.valid = {'right', 'undo'}
#
#     def update(self, action, traj_gen):
#         if action in self.valid:
#             state = self._check_state(traj_gen.ego_pose.position, traj_gen._kp2)
#             if state == 0:
#                 return 'mr1', traj_gen.follow_phase_1()
#             else:
#                 return traj_gen.acc.update(action, traj_gen)
#         else:
#             return 'fail', []


# class Undo(State):
#
#     def __init__(self):
#         self.valid = {'undo'}
#
#     def update(self, action, traj_gen):
#         if action in self.valid:
#             state = self._check_state(traj_gen.ego_pose.position, traj_gen._kp2)
#             if state == 0:
#                 return 'undo', traj_gen.follow_phase_1()
#             else:
#                 return traj_gen.acc.update(action, traj_gen)
#         else:
#             return 'fail', []


class Fail(State):

    def __init__(self):
        self.valid = {'acc', 'left', 'right', 'undo'}

    def update(self, action, traj_gen):
        raise Exception("Finite State Machine Has Failed on", action)


class TrajGenerator(object):

    def __init__(self, tsmap_handler):
        self.acc = Acc()
        self.m0 = Merge0()
        self.m1 = Merge1()
        self.fail = Fail()

        self.state = self.acc

        self.map_handler = TrajMap(tsmap_handler)

        self._kp1 = None
        self._kp2 = None
        self.v_lateral = 0.5
        self.cancel_rate = 2.0
        self.adj_lim = 0.3
        self.change_x_speed = self.map_handler.lane_width / 4

        self.min_forward = 2.0

        self.full_profile = None
        self.ego_x = None
        self.ego_y = None
        self.t_interval = None
        self.acc_shift = None
        self.central_distance = None
        self.merge_distance = None

    def generate(self, action, speed_profile, ego_pose):
        # TODO: ego_pose contains time stamp and speed_profile contains global time
        # Now, I take speed profile as [[v1, v2, ..., vn], t_interval, init_t]
        # len vs must > 1
        vs, self.t_interval, init_t = speed_profile
        self.ego_x, self.ego_y = ego_pose.position
        self.acc_shift = self.v_lateral * self.t_interval
        self.full_profile = self._expand_profile(vs, ego_pose.speed)
        new_state, points = self.state.update(action, self)
        self.state = eval('self.' + new_state)
        if new_state == 'm0' or new_state == 'm1':
            self.state.set_action(action)
        trajectory = self._generate_traj(points)
        if new_state == 'acc':
            st = True
        else:
            st = False
        return trajectory, st

    def _generate_traj(self, points):
        traj = []
        for index in xrange(len(points) - 1):
            x, y, heading = points[index]
            nx, ny, nheading = points[index + 1]
            traj.append({'pose': {'position': [x, y], 'speed': math.sqrt((nx - x) ** 2 + (ny - y) ** 2) / self.t_interval, 'heading': heading, 'curvature': 0.0, 'jerk': 0.0},
                          'time': self.full_profile[index][2]})
        for i in xrange(len(traj) - 1):
            traj[i]['pose']['acceleration'] = (traj[i + 1]['pose']['speed'] - traj[i]['pose']['speed']) / self.t_interval
        traj[-1]['pose']['acceleration'] = traj[-2]['pose']['acceleration']


        return traj

    def _expand_profile(self, vs, speed):
        full_profile = []
        dist = 0
        t = 0
        v_l = speed
        for v in vs:
            dist += (v_l + v) * self.t_interval / 2.0
            t += self.t_interval
            v_l = v
            full_profile.append((dist, v, t))
        # print "profile: ", full_profile
        return full_profile

    def generate_acc(self, drct, thresh=0):
        centrals = [p[0] for p in self.full_profile]
        points, _ = self.map_handler.navigate_by_centrals(self.ego_x, self.ego_y,
                                              centrals, self.acc_shift, drct, thresh)
        self._fix_heading(points, range(len(points) - 1))
        return points

    def generate_merge(self, action, thresh=0):
        idx1 = self._get_turn_point_index(self.min_forward)
        centrals = [p[0] for p in self.full_profile[: idx1 + 1]]
        if action == 'left':
            drct = -1
        else:
            drct = 1
        points1, d = self.map_handler.navigate_by_centrals(self.ego_x, self.ego_y, centrals, self.acc_shift, drct,
                                                           thresh)
        self._kp1 = points1[-1]
        lateral_distance = self.map_handler.lane_width - drct * d
        central_distance = self._get_merge_distance(lateral_distance, self.full_profile[idx1][1])
        idx2 = self._get_turn_point_index(centrals[-1] + central_distance)
        self.central_distance = self.full_profile[idx2][0] - centrals[-1]
        self._kp2 = self.map_handler.get_correspondent_point(self.ego_x, self.ego_y, drct, self.full_profile[idx2][0])
        self.merge_distance = math.sqrt((self._kp2[0] - self._kp1[0]) ** 2 + (self._kp2[1] - self._kp1[1]) ** 2)
        trans_points = self._connect(self._kp1, self._kp2, self.full_profile[idx1 + 1: idx2], self.full_profile[idx1][0])
        centrals2 = [p[0] - self.full_profile[idx2][0] for p in self.full_profile[idx2:]]
        points2, _ = self.map_handler.navigate_by_centrals(self._kp2[0], self._kp2[1], centrals2, self.acc_shift, 0)
        rslts = points1 + trans_points + points2
        base = len(points1) + len(trans_points) - 1
        self._fix_heading(rslts, range(len(points1)) + [base + i for i in range(len(points2))])
        return rslts
        # return points1[:-1] + trans_points + points2[1:]

    def _get_turn_point_index(self, distance):
        for index in xrange(len(self.full_profile)):
            if self.full_profile[index][0] >= distance:
                return index
        d_base, v, t_base = self.full_profile[-1]
        ls = v * self.t_interval
        num = int(math.ceil((distance - d_base) / ls))
        for i in xrange(num):
            d_base += ls
            t_base += self.t_interval
            self.full_profile.append([d_base, v, t_base])
        return len(self.full_profile) - 1

    def _get_merge_distance(self, lateral_distance, y_speed):
        return y_speed * (lateral_distance / self.change_x_speed)

    def _connect(self, point1, point2, dvt, base):
        # point1, point2 exclusive
        # every point in dvt, inclusive
        x1, y1, _ = point1
        x2, y2, _ = point2
        heading = math.atan2(y2 - y1, x2 - x1)
        delta_x = math.cos(heading)
        delta_y = math.sin(heading)
        points = []
        # print "total dist: ", math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
        # print "delta_x: ", delta_x, "delta_y: ", delta_y
        ratio = self.merge_distance / self.central_distance
        for index in xrange(len(dvt)):
            central_dist = dvt[index][0] - base
            actual_dist = central_dist * ratio
            points.append([x1 + actual_dist * delta_x, y1 + actual_dist * delta_y, heading])
        return points

    def follow_phase_0(self):
        # in this function, we have central_distance \approx actual_distance for simplicity
        # only dist applies this approximation
        x1, y1, heading1 = self._kp1
        dist = math.sqrt((self.ego_x - x1) ** 2 + (self.ego_y - y1) ** 2)
        heading = math.atan2(y1 - self.ego_y, x1 - self.ego_x)
        idx1 = self._get_turn_point_index(dist)
        idx2 = self._get_turn_point_index(dist + self.central_distance)
        points1 = []
        for i in xrange(idx1):
            d = self.full_profile[i][0]
            points1.append(((d * x1 + (dist - d) * self.ego_x) / dist, (d * y1 + (dist - d) * self.ego_y) / dist, heading))
        points1.append(self._kp1)
        points2 = self._connect(self._kp1, self._kp2, self.full_profile[idx1 + 1: idx2], dist)
        centrals2 = [self.full_profile[i][0] - dist - self.central_distance for i in xrange(idx2, len(self.full_profile))]
        points3, lateral = self.map_handler.navigate_by_centrals(self._kp2[0], self._kp2[1], centrals2, self.acc_shift, 0)
        base = len(points1) + len(points2) - 1
        rslts = points1 + points2 + points3
        self._fix_heading(rslts, [base + i for i in range(len(points3))])
        return rslts
        # return points1[:-1] + points2 + points3[1:]

    def follow_phase_1(self):
        x1, y1, heading1 = self._kp2
        ratio = self.merge_distance / self.central_distance
        c_dist = math.sqrt((self.ego_x - x1) ** 2 + (self.ego_y - y1) ** 2) / ratio
        heading = math.atan2(y1 - self.ego_y, x1 - self.ego_x)
        idx1 = self._get_turn_point_index(c_dist)
        delta_x = math.cos(heading)
        delta_y = math.sin(heading)
        points1 = []
        for index in xrange(idx1):
            dist = self.full_profile[index][0] * ratio
            points1.append([self.ego_x + dist * delta_x, self.ego_y + dist * delta_y, heading])
        centrals = [self.full_profile[i][0] - c_dist for i in xrange(idx1, len(self.full_profile))]
        points2, lateral = self.map_handler.navigate_by_centrals(x1, y1, centrals, self.acc_shift, 0)
        rslts = points1 + points2
        base = len(points1) - 1
        self._fix_heading(rslts, [base + i for i in range(len(points2))])
        return rslts
        # return points1 + points2[1:]

    def undo_merge(self):
        dst_to_kp1 = math.sqrt((self.ego_x - self._kp1[0]) ** 2 + (self.ego_y - self._kp1[1]) ** 2)
        central_used = self.central_distance * dst_to_kp1 / self.merge_distance
        central_back = central_used / self.cancel_rate
        self._kp2 = self.map_handler.get_correspondent_point(self._kp1[0], self._kp1[1], 0, central_used + central_back)
        idx2 = self._get_turn_point_index(central_back)
        trans_points = self._connect((self.ego_x, self.ego_y, 0), self._kp2, self.full_profile[:idx2], 0)
        centrals2 = [self.full_profile[i][0] - central_back for i in xrange(idx2, len(self.full_profile))]
        points2, _ = self.map_handler.navigate_by_centrals(self._kp2[0], self._kp2[1], centrals2, self.acc_shift, 0)
        rslts = trans_points + points2
        base = len(trans_points) - 1
        self._fix_heading(rslts, [base + i for i in range(len(points2))])
        return rslts
        # return trans_points + points2[1:]

    @staticmethod
    def _fix_heading(points, idxs):
        for idx in idxs:
            points[idx][2] = math.atan2(points[idx+1][1] - points[idx][1], points[idx+1][0] - points[idx+1][0])


if __name__ == '__main__':
    map_path = '/mnt/truenas/scratch/data/map/hdmap/data/MAP_new/torrey.hdmap'
    class T:
        def __init__(self, x, y, s):
            self.position = (x, y)
            self.speed = s
    with open(map_path, 'rb') as f:
        submap = f.read()
    tsmap = TSMap.deserialize(submap)
    # print type(tsmap)
    traj_generator = TrajGenerator(tsmap)
    ego_pose = T(-307076.53037816077, 28977.825006679916, 10)
    speed_list = [(10 + 0.05 * i, (i + 1.0) / 10) for i in range(3)] + [(11.5 - 0.05 * i, (3.0 + i) / 10) for i in range(3)]
    speed_profile = [[10 + 0.005 * i for i in range(30)] + [10.15 - 0.005 * i for i in range(30)], 0.1, 0.0]
    ACC = 'acc'
    MERGE_RIGHT = 'right'
    MERGE_LEFT = 'left'
    EMERGENCY = 'undo'
    actions = [ACC] * 50 + [MERGE_RIGHT] * 10 + [EMERGENCY] + [ACC] * 30 + [MERGE_LEFT] + [ACC] * 30
    # actions = [ACC, ACC, ACC, MERGE_RIGHT, EMERGENCY, MERGE_RIGHT, ACC, ACC, MERGE_LEFT, EMERGENCY, ACC, ACC, MERGE_LEFT, ACC]
    # actions = [ACC, ACC, ACC, MERGE_RIGHT, MERGE_RIGHT, EMERGENCY, MERGE_RIGHT, ACC, ACC, MERGE_LEFT, MERGE_LEFT, EMERGENCY, ACC, ACC,
    #           MERGE_LEFT, ACC]

    sample_number = 50
    index = 0
    num_actions = len(actions)
    kpx = []
    kpy = []
    kp3x = []
    kp3y = []
    kp4x = []
    kp4y = []
    positions = []
    times = 0
    alltimemax = 0
    while index < num_actions:
        print "To ", actions[index], ", pos: ", ego_pose.position
        trajs, status = traj_generator.generate(actions[index], speed_profile, ego_pose)
        print "action: ", actions[index], ", traj: ", len(trajs)
        ego_pose = T(trajs[2]['position'][0], trajs[2]['position'][1], trajs[2]['speed'])
        positions.append([trajs[0]['position'], trajs[0]['heading'], trajs[0]['speed']])
        positions.append([trajs[1]['position'], trajs[1]['heading'], trajs[1]['speed']])
        positions.append([trajs[2]['position'], trajs[2]['heading'], trajs[2]['speed']])

        # if status or (index + 1 < num_actions and 'merge' in actions[index + 1]):
        if not status:
            times += 1
        else:
            times = 0
        if status or (times > 1 and index + 1 < num_actions and actions[index + 1] == EMERGENCY):
            # if 'merge' in actions[index]:
            #     kpx.append(traj_generator._kp1[0])
            #     kpy.append(traj_generator._kp1[1])
            #     kpx.append(traj_generator._kp2[0])
            #     kpy.append(traj_generator._kp2[1])
            #     kp3x.append(traj_generator._kp3[0])
            #     kp3y.append(traj_generator._kp3[1])
            #     kp4x.append(traj_generator._kp4[0])
            #     kp4y.append(traj_generator._kp4[1])
            index += 1
    # import pickle
    # with open('emer.pkl', 'wb') as w:
    #     pickle.dump(positions, w, pickle.HIGHEST_PROTOCOL)
    print alltimemax
    import numpy as np
    import matplotlib.pyplot as plt
    positions = np.array(positions)
    xs = positions[:, 0]
    ys = positions[:, 1]
    lane1 = tsmap.get_lane(Point3d(-307076.53037816077, 28977.825006679916, 0))
    lane2 = lane1.right_lane
    pts1 = []
    pts2 = []
    for ref in lane1.ref_pts:
        pts1.append([ref.x, ref.y])
    for ref in lane2.ref_pts:
        pts2.append([ref.x, ref.y])
    pts1 = np.array(pts1)[:5000]
    pts2 = np.array(pts2)[:5000]
    plt.plot(xs,ys)
    plt.plot(pts1[:, 0], pts1[:, 1])
    plt.plot(pts2[:, 0], pts2[:, 1])
    # plt.plot(kpx, kpy)
    # plt.plot(kp3x, kp3y)
    # plt.plot(kp4x, kp4y)
    plt.show()