from collections import deque

import debugging_tools.octopus_logger as oclog
import matplotlib.pyplot as plt
import numpy as np
import utilities.geometry as ge
from sklearn.linear_model import Ridge

from src.unused.pose import Pose

DEFAULT_SAMPLING_NUMBER = 200
SMALL_NUMBER = 0.001
DEFAULT_TIME_HORIZON = 5.0
MINIMUM_EXTEND_STEP = 1  # when trajectory is not long enough and the speed is 0. Use it the extend the trajectory
DEFAULT_MAX_ACCELERATION = 3.
DEFAULT_MAX_DECELERATION = -5.


class Trajectory:
    def __init__(self, poses, time_step, perception_time_stamp):
        self.way_points = poses
        self.time_step = time_step
        self.start_time = perception_time_stamp

    def to_planner_result(self):
        """
        convert the trajectory to a list of Pose objects
        :return: a list of Pose objects that represents the waypoints for the car to follow.
        """
        planner_result = {'perceptionTimeStamp': time_stamp, 'trajectory': []}
        for waypoint in self.way_points:
            time_step = self.planner_setting.time_horizon / self.planner_setting.temporal_sampling_number
            planner_result['trajectory'].append({
                'time': time_stamp + t * time_step,
                'pose': waypoint.to_dict()
            })
        return planner_result

    def get_poses(self):
        return self.way_points



class BasicPlannerPath(Path):
    def __init__(self, path_segments):
        """

        :param path_segments: a list of Curve objects which composes the path
        """
        if path_segments is None:
            oclog.warn("Path segment is None")
            raise Exception('Path segment is None')
        elif len(path_segments) == 0:
            oclog.warn("Path with empty path segments created")
            raise Exception('Path with empty path segments created')
        self.path_segments = path_segments
        self.path_length = 0.0
        for path_segment in self.path_segments:
            self.path_length += path_segment.get_length()

    def get_pose_at_distance(self, path_distance):
        if path_distance == self.get_length():
            return self.path_segments[-1].get_pose_at_distance(self.path_segments[-1].get_length())
        elif path_distance > self.get_length():
            oclog.warn("try to get the pose out of the range of the path: {}, {}. No exception is raised".format(
                path_distance, self.get_length()))
            return self.path_segments[-1].get_pose_at_distance(self.path_segments[-1].get_length())
        for i in range(len(self.path_segments)):
            if path_distance > self.path_segments[i].get_length():
                path_distance -= self.path_segments[i].get_length()
            else:
                return self.path_segments[i].get_pose_at_distance(path_distance)

        # raise Exception('Attempt to get a point out of the path length')

    def get_pose_at_distance_list(self, path_distance_list):
        """
        Find the segment for each path_distance's final position and retrieve pose in that segment.
        Assume path_distance_list is in INCREASING order!
        :param path_distance_list:
        :return:
        """

        if path_distance_list[-1] > self.get_length():
            oclog.warn("max_distance: " + str(path_distance_list[-1]))
            oclog.warn("distance_range: " + str(self.get_length()))
            oclog.warn("try to get the pose out of the range of the path. No exception is raised")

            # raise Exception("try to get the pose out of the range of the path")
        distance_list_in_segments = [[] for _ in range(len(self.path_segments))]
        # split the distance list into several sublist with different segments
        accumulated_length = 0
        path_segment_index = 0
        for i in range(len(self.path_segments)):
            accumulated_length += self.path_segments[i].get_length()
            # add all path distances that are less than accumulated length into current path segment list
            while path_segment_index < len(path_distance_list) and  \
                    path_distance_list[path_segment_index] <= accumulated_length:
                distance_list_in_segments[i].append(path_distance_list[path_segment_index] -
                                                    accumulated_length + self.path_segments[i].get_length())
                path_segment_index += 1

        res = []
        # get pose of each distance in different segments
        for i in range(len(self.path_segments)):
            if len(distance_list_in_segments[i]) > 0:
                res.extend(self.path_segments[i].get_pose_at_distance_list(distance_list_in_segments[i]))
        return res

    def get_length(self):
        """
        get the length of a path
        :return:
        """
        return self.path_length

    def plot(self):
        """
        Plot the path. Used for debugging.
        :return:
        """
        positions = []
        for s in np.linspace(0, self.get_length(), DEFAULT_SAMPLING_NUMBER):
            positions.append(self.get_pose_at_distance(s).position)
        xy = np.swapaxes(np.array(positions), 0, 1)
        plt.figure(1)
        plt.plot(xy[0], xy[1], 'b--')
        plt.show()


class BasicPlannerTrajectory(Trajectory):
    def __init__(self, path, speed_profile):
        self.path = path
        self.speed_profile = speed_profile

    def get_pose_at_time(self, trajectory_time):
        s = self.speed_profile.get_distance_at_time(trajectory_time)
        if s < 0:
            oclog.warn("trajectory distance is negative at input trajectory time")
            oclog.warn("s: " + str(s))
            oclog.warn("trajectory_time: " + str(trajectory_time))
            oclog.warn("speed_profile.speed: " + str(self.speed_profile.speed))
            raise Exception("trajectory distance is negative at input trajectory time")
        p = self.path.get_pose_at_distance(s)
        v = self.speed_profile.get_speed_at_time(trajectory_time)
        p.speed = v[0]
        p.acceleration = v[1]
        p.jerk = 0
        return p

    def get_pose_at_time_list(self, trajectory_time_list):
        """
        the length of returned poses will be smaller than that of time_list when the path length can't cover horizon
        :param trajectory_time_list:
        :return:
        """
        trajectory_distance_list = []
        for t in trajectory_time_list:
            trajectory_distance_list.append(self.speed_profile.get_distance_at_time(t))
        poses = self.path.get_pose_at_distance_list(trajectory_distance_list)
        for i in range(len(poses)):
            v = self.speed_profile.get_speed_at_time(trajectory_time_list[i])
            poses[i].speed = v[0]
            poses[i].acceleration = v[1]
            poses[i].jerk = 0
        return poses

    def get_time_horizon(self):
        return self.speed_profile.get_time_horizon()

    def to_planner_result(self, horizon=DEFAULT_TIME_HORIZON,
                          temporal_sampling_number=DEFAULT_SAMPLING_NUMBER,
                          is_extend_trajectory=True):
        """
        :param horizon: time horizon
        :param temporal_sampling_number: number of points
        :param is_extend_trajectory: if this parameter is true, extend the trajectory
        :return: satisfied trajectory
        """
        time_list = np.linspace(0, horizon, temporal_sampling_number)
        result = self.get_pose_at_time_list(time_list)
        if not is_extend_trajectory:
            return result
        # Extend the trajectory
        distance = self.get_dis_at_time(time_list[-1])
        path_len = self.path.get_length()
        distance_list = np.linspace(distance, path_len, temporal_sampling_number)
        oclog.debug("Extend the trajectory. Current: " + str(distance) + ". Extend to " + str(self.path.get_length()))
        speed = result[-1].speed
        pose_list = self.path.get_pose_at_distance_list(distance_list)
        for pose in pose_list:
            pose.speed = speed
            pose.acceleration = 0
            pose.jerk = 0
        result.extend(pose_list[1:])  # avoid adding the last point of trajectory twice
        oclog.debug("number of pose: " + str(len(result)))
        return result

    def transform_trajectory_with_respect_to_pose(self, pose):
        zero_pose = [pose.position[0], pose.position[1], pose.heading]
        heading_sin = np.sin(pose.heading)
        heading_cos = np.cos(pose.heading)
        transformed_path_segments = []
        for path_segment in self.path.path_segments:
            transformed_start_position = \
                ge.get_local_point([path_segment.start_position[0], path_segment.start_position[1],
                                    path_segment.start_heading], zero_pose, heading_sin, heading_cos)
            transformed_end_position = \
                ge.get_local_point([path_segment.end_position[0], path_segment.end_position[1],
                                    0], zero_pose, heading_sin, heading_cos)
            transformed_path_segment = PolyCurve(poly_curve=path_segment)
            transformed_path_segment.start_position = transformed_start_position[:2]
            transformed_path_segment.end_position = transformed_end_position[:2]
            transformed_path_segment.start_heading = transformed_start_position[2]
            transformed_path_segment.end_heading = transformed_end_position[2]
            transformed_path_segment.cache_reset()
            transformed_path_segments.append(transformed_path_segment)

        self.path.path_segments = transformed_path_segments

    def plot(self, horizon=DEFAULT_TIME_HORIZON, temporal_sampling_number=DEFAULT_SAMPLING_NUMBER):
        """
        Plot the trajectory. Used for debugging
        :param horizon: 0 <= horizon <= self.get_time_horizon. The maximum time in the plot. Default value is 3
        :param temporal_sampling_number: The bigger the temporal_sampling_number, the better the plot. Default value is 100
        :return:
        """
        poses = self.to_planner_result(horizon, temporal_sampling_number)
        positions = [x.position for x in poses]
        xy = np.swapaxes(np.array(positions), 0, 1)
        plt.figure(2)
        plt.plot(xy[0], xy[1], 'r--')
        plt.show()

    def get_dis_at_time(self, trajectory_time):
        return self.speed_profile.get_distance_at_time(trajectory_time)


class ACCPlannerSpeedProfile:
    """
    Objects in this class describe speed profiles of a dynamic object. Support odometer query and speed query at any 0 <= t <= self.time_horizon
    This class assumes the dynamic has a piecewise velocity in the time horizon and following the front car.
    """
    def __init__(self, speed, alpha, target_speed, target_displacement, speed_profile_time_horizon, sampling_number):
        """
        :param speed: the initial speed
        :param acceleration: the acceleration
        :param speed_profile_time_horizon: the time horizon of the speed profile
        """
        self.alpha = alpha
        self.speed = speed * 1.
        self.target_speed = target_speed * 1.
        self.speed_profile_time_horizon = speed_profile_time_horizon * 1.
        self.sampling_number = sampling_number
        self.time_sample = speed_profile_time_horizon * 1. / sampling_number
        self.target_displacement = target_displacement

        """
        this function is the core part for optimization
        :param:
        :return: a series of thrusts(accelerations) at the sampling time points
        """

        def get_coeffient_vector(n, state_init, state_end, ts):
            return np.array([state_end[0] - state_init[0], state_end[1] - state_init[1] - ts * n * state_init[0]])

        def get_coeffient_matrix(n, ts):
            return np.array([[ts] * n, list(ts ** 2 * np.arange(n - 1., -1., -1))])

        def get_state(idx, u, z0, ts):
            # TODO: can do acceleration using z_{idx-1}
            state = np.array([z0[0], ts * idx * z0[0] + z0[1]])
            for i in range(idx):
                state += u[i] * np.array([ts, ts ** 2 * (idx - i - 1)])
            return state

        self.state_init = np.array([self.speed, 0])
        self.state_end = np.array([self.target_speed, self.target_displacement])
        P = get_coeffient_matrix(self.sampling_number, self.time_sample) # coeffient matrix
        b = get_coeffient_vector(self.sampling_number, self.state_init, self.state_end, self.time_sample)
        solver = Ridge(alpha=alpha, fit_intercept=False)
        solver.fit(P, b) # solve Pa = b
        self.accelerations = list(solver.coef_) + [0]

        states_speed = [self.state_init[0]]
        states_distance = [self.state_init[1]]
        for i in np.arange(1, self.sampling_number + 1):
            statei = get_state(i, solver.coef_, self.state_init, self.time_sample)
            states_speed.append(statei[0])
            states_distance.append(statei[1])
        #self.time_stamp = np.arange(0., self.speed_profile_time_horizon + self.time_sample, self.time_sample)
        self.speeds = states_speed
        self.distances = states_distance


    def get_distance_at_time(self, speed_profile_time):
        """
        get the odometer of the dynamic object at time t
        :param speed_profile_time: time
        :return: odometer
        """
        time_stamp_index = int(np.floor(speed_profile_time / .2))
        return self.distances[time_stamp_index]

    def get_speed_at_time(self, speed_profile_time):
        """
        Get speed and acceleration of the dynamic object at time t
        :param t: time
        :return: [speed, acceleration]
        """
        time_stamp_index = int(np.floor(speed_profile_time / .2))
        return [self.speeds[time_stamp_index], self.accelerations[time_stamp_index]]

    def get_time_horizon(self):
        return self.speed_profile_time_horizon


class BasicPlannerSpeedProfile:
    """
    Objects in this class describe speed profiles of a dynamic object. Support odometer query and speed query at any 0 <= t <= self.time_horizon
    This class assumes the dynamic has a constant acceleration in the time horizon and does not move backward.
    """
    def __init__(self, speed, acceleration, speed_profile_time_horizon):
        """

        :param speed: the initial speed
        :param acceleration: the acceleration
        :param speed_profile_time_horizon: the time horizon of the speed profile
        """
        self.speed = speed
        self.acceleration = acceleration
        self.speed_profile_time_horizon = speed_profile_time_horizon


    def get_distance_at_time(self, speed_profile_time):
        """
        get the odometer of the dynamic object at time t
        :param speed_profile_time: time
        :return: odometer
        """
        # ensure s >= 0:
        if speed_profile_time > self.speed_profile_time_horizon:
            oclog.warn("Speed profile time exceeds total time")
            oclog.warn("access time: " + str(speed_profile_time))
            oclog.warn("speed profile length: " + str(self.speed_profile_time_horizon))
            raise Exception("Speed profile time exceeds total time")
        if self.acceleration < 0:
            duration = min(speed_profile_time, -self.speed / self.acceleration)
        else:
            duration = speed_profile_time
        return self.speed * duration + self.acceleration * duration ** 2 / 2.0

    def get_speed_at_time(self, speed_profile_time):
        """
        Get speed and acceleration of the dynamic object at time t
        :param t: time
        :return: [speed, acceleration]
        """
        if speed_profile_time > self.speed_profile_time_horizon:
            oclog.warn("Speed profile time exceeds total time")
            raise Exception("Speed profile time exceeds total time")
        return [max(self.speed + self.acceleration * speed_profile_time, 0.0), self.acceleration]

    def get_time_horizon(self):
        return self.speed_profile_time_horizon


class QuadraticPlannerSpeedProfile:
    """ in this class describe speed profiles of a dynamic object.
    This class assumes the dynamic tries to sample a smooth connected speed profile. Besides, it is sampling with different jerk.
    """
    def __init__(self, pose, speed_profile_time_horizon, jerk):
        """
        :param pose: the initial pose
        :param previous quadratic parameters: a,b,c,(at^2+bt+c)
        :param speed_profile_time_horizon: the time horizon of the speed profile
        :param jerk: sampled jerk
        """
        oclog.debug('generating quadratic speed profile: {}, {}, {}'.format(str(pose.speed), str(speed_profile_time_horizon), str(jerk)))
        self.pose = pose
        # ASSUME SPEED_PROFILE_TIME_HORIZON IS CONSTANT
        # self.prev_quad_paras = (pose.jerk * 0.5, pose.acceleration - pose.jerk * speed_profile_time_horizon, pose.speed)
        self.speed_profile_time_horizon = speed_profile_time_horizon
        # if self.pose.speed > speed_range[1]:
        #     self.jerk = -0.1
        # else:
        #     self.jerk = jerk
        self.jerk = jerk
        self.cur_quad_paras = [self.jerk * 0.5, self.pose.acceleration, self.pose.speed]


    def nonnegative_speed(self):
        """
        whether speed is non negative
        :return if speed has negative value, return 0, else return 1
        """
        a = self.cur_quad_paras[0]
        b = self.cur_quad_paras[1]
        c = self.cur_quad_paras[2]
        ts = self.speed_profile_time_horizon
        if np.abs(a) > np.spacing(1):
            if a * ts**2 + b * ts + c >= 0 and -b**2 *(0.25 / np.abs(a) - 0.5 / a) + c >= 0:
                return 1.0
            else:
                return 0.
        else: # linear function
            if b < 0 and b * ts + c < 0:
                return 0.
            else:
                return 1.0

    def estimate_acceleration(self,maxacc = DEFAULT_MAX_ACCELERATION, minacc = DEFAULT_MAX_DECELERATION):
        """
        decide if acceleration is out of range
        :param maxacc: the max acceleration of the vehicle
        :param minacc: negative, the max deceleration of the vehicle
        :return: whether over maxacc and whether over minacc
        """
        startacc = self.cur_quad_paras[1]
        endacc = self.cur_quad_paras[1] + 2*self.cur_quad_paras[0]*self.speed_profile_time_horizon
        if startacc > maxacc or endacc > maxacc:
            overmaxacc = 1.0
        else:
            overmaxacc = 0
        if startacc < minacc or endacc < minacc:
            overminacc = 1.0
        else:
            overminacc = 0
        return (overmaxacc, overminacc)

    def get_distance_at_time(self, t, maxacc = DEFAULT_MAX_ACCELERATION, minacc = DEFAULT_MAX_DECELERATION):
        """
        get the odometer of the dynamic object at time t
        :param t: time
        :return: odometer
        """
        cur_quad_paras = self.cur_quad_paras
        return (1.0 / 3 * cur_quad_paras[0] * t ** 3 + 0.5 * cur_quad_paras[1] * t ** 2 + cur_quad_paras[2] * t)
        # TODO the following comment is of the stable version
        # if t > self.speed_profile_time_horizon:
        #     oclog.warn("Speed profile time exceeds total time")
        #     oclog.warn("access time: " + str(t))
        #     oclog.warn("speed profile length: " + str(self.speed_profile_time_horizon))
        #     raise Exception("Speed profile time exceeds total time")
        # if self.estimate_acceleration() == (0,0):
        #     return cur_quad_paras[2] * t * (1 - self.nonnegative_speed()) + \
        #     (1.0/3 * cur_quad_paras[0] * t**3 + 0.5 * cur_quad_paras[1] * t**2 + cur_quad_paras[2] * t) * self.nonnegative_speed()
        # elif self.estimate_acceleration()[1] == 1:
        #     self.cur_quad_paras[0] = 0
        #     self.cur_quad_paras[1] = minacc
        #     return 0.5*self.cur_quad_paras[1]*t**2 + self.cur_quad_paras[2] * t
        # else:
        #     self.cur_quad_paras[0] = 0
        #     self.cur_quad_paras[1] = maxacc
        #     return 0.5*self.cur_quad_paras[1]*t**2 + self.cur_quad_paras[2] * t

    def get_speed_at_time(self, t, maxacc = DEFAULT_MAX_ACCELERATION, minacc = DEFAULT_MAX_DECELERATION):
        """
        Get speed and acceleration of the dynamic object at time t
        :param t: speed_profile_time
        :return: [speed, acceleration]
        """
        cur_quad_paras = self.cur_quad_paras
        if t > self.speed_profile_time_horizon:
            oclog.warn("Speed profile time exceeds total time")
            raise Exception("Speed profile time exceeds total time")
        return [(cur_quad_paras[0] * t ** 2 + cur_quad_paras[1] * t + cur_quad_paras[2]), (2 * cur_quad_paras[0] * t + cur_quad_paras[1])]
        # TODO the following comment is of the stable version
        # decider = self.nonnegative_speed()
        # if self.estimate_acceleration() == (0, 0):
        #     return [(cur_quad_paras[0] * t**2 + cur_quad_paras[1] * t + cur_quad_paras[2]) * decider + cur_quad_paras[2] * (1 - decider),
        #         (2 * cur_quad_paras[0] * t + cur_quad_paras[1]) * decider]
        # elif self.estimate_acceleration()[1] == 1:
        #     self.cur_quad_paras[0] = 0
        #     self.cur_quad_paras[1] = minacc
        #     return [self.cur_quad_paras[1]*t + self.cur_quad_paras[2], self.cur_quad_paras[1]]
        # else:
        #     self.cur_quad_paras[0] = 0
        #     self.cur_quad_paras[1] = maxacc
        #     return [self.cur_quad_paras[1]*t + self.cur_quad_paras[2],cur_quad_paras[2], self.cur_quad_paras[1]]

    def get_time_horizon(self):
        return self.speed_profile_time_horizon


class ComplexSpeedProfile:
    def __init__(self, speed, accelerate_list, time_list):
        if len(accelerate_list) != len(time_list):
            oclog.warn("acceleration and time interval are not of the same length")
            raise Exception("acceleration and time interval are not of the same length")
        self.speed = speed
        self.accelerate_list = accelerate_list
        self.time_list = time_list
        self.speed_profile_time = sum(self.time_list)

    def get_distance_at_time(self, speed_profile_time):
        # if speed_profile_time > self.speed_profile_time:
        #     raise Exception("Speed profile time exceeds total time")
        distance = 0.0
        index = 0
        current_speed = self.speed
        current_time = speed_profile_time
        while current_time - self.time_list[index] > SMALL_NUMBER:
            if self.accelerate_list[index] < 0:
                duration = min(self.time_list[index], -current_speed / self.accelerate_list[index])
            else:
                duration = self.time_list[index]
            distance += current_speed * duration + self.accelerate_list[index] * duration ** 2 / 2.0
            current_speed += self.accelerate_list[index] * duration
            current_time -= self.time_list[index]
            index += 1
            if index == len(self.time_list):
                oclog.warn("Time horizon is not enough. Use longer time horizon in config file.")
                oclog.warn("speed profile time: " + str(speed_profile_time))
                oclog.warn("speed profile horizon: " + str(self.speed_profile_time))
                raise Exception("Time horizon is not enough. Use longer time horizon in config file.")
        if self.accelerate_list[index] < 0:
            duration = min(current_time, -current_speed / self.accelerate_list[index])
        else:
            duration = current_time
        distance += current_speed * duration + self.accelerate_list[index] * duration ** 2 / 2.0
        return distance

    def get_speed_at_time(self, speed_profile_time):
        if speed_profile_time > self.speed_profile_time:
            oclog.warn("Speed profile time exceeds total time")
            raise Exception("Speed profile time exceeds total time")
        current_speed = self.speed
        current_time = speed_profile_time
        index = 0
        while current_time - self.time_list[index] > SMALL_NUMBER:
            current_speed += self.accelerate_list[index] * self.time_list[index]
            current_speed = max(current_speed, 0)
            current_time -= self.time_list[index]
            index += 1
        current_speed += self.accelerate_list[index] * current_time
        current_speed = max(current_speed, 0)
        return [current_speed, self.accelerate_list[index]]

    def get_time_horizon(self):
        return self.speed_profile_time


class PartialTrajectory(Trajectory):
    def __init__(self, previous_trajectory, current_trajectory, perception_gap_time, time_bound):
        if not isinstance(previous_trajectory, PartialTrajectory):
            # if previous trajectory is a single trajectory
            if previous_trajectory.get_time_horizon() < perception_gap_time + time_bound:
                oclog.warn("previous trajectory is not long enough to get partial trajectory!")
                raise Exception("previous trajectory is not long enough to get partial trajectory!")
            trajectory_deque = deque([previous_trajectory])
            time_deque = deque([[perception_gap_time, time_bound]])
        else:
            if perception_gap_time == 0:
                self.trajectory_deque = previous_trajectory.trajectory_deque
                self.time_deque = previous_trajectory.time_deque
                self.time_horizon = previous_trajectory.time_horizon
            trajectory_deque = previous_trajectory.trajectory_deque
            time_deque = previous_trajectory.time_deque
            if len(trajectory_deque) == 0:
                oclog.warn("previous trajectory is empty")
                raise Exception("previous trajectory is empty")
            while len(trajectory_deque) > 0:
                if time_deque[0][1] <= perception_gap_time:
                    perception_gap_time -= time_deque[0][1]
                    trajectory_deque.popleft()
                    time_deque.popleft()
                else:
                    time_deque[0][0] += perception_gap_time
                    time_deque[0][1] -= perception_gap_time
                    break
            i = 0
            while True:
                if time_bound >= time_deque[i][1]:
                    time_bound -= time_deque[i][1]
                    i += 1
                    if i == len(time_deque) and time_bound > 0:
                        oclog.warn("previous trajectory is not long enough to get partial trajectory!")
                        raise Exception("previous trajectory is not long enough to get partial trajectory!")
                else:
                    break
            time_deque[i][1] = time_bound

        trajectory_deque.append(current_trajectory)
        time_deque.append([0, current_trajectory.get_time_horizon()])
        self.trajectory_deque = trajectory_deque
        self.time_deque = time_deque
        self.time_horizon = 0
        for i in range(len(self.time_deque)):
            self.time_horizon += self.time_deque[i][1]

    def get_pose_at_time(self, trajectory_time):
        index, trajectory_time = self.get_deque_index_at_time(trajectory_time)
        return self.trajectory_deque[index].get_pose_at_time(self.time_deque[index][0] + trajectory_time)

    def get_pose_at_time_list(self, trajectory_time_list):
        if trajectory_time_list[-1] > self.time_horizon:
            oclog.warn("try to get the pose out of the range of the path")
            raise Exception("try to get the pose out of the range of the path")
        time_list_in_segments = [[] for _ in range(len(self.time_deque))]
        # split the distance list into several sublist with different segments
        accumulated_time = 0
        time_segment_index = 0
        for i in range(len(self.time_deque)):
            accumulated_time += self.time_deque[i][1]
            # add all path distances that are less than accumulated length into current path segment list
            while time_segment_index < len(trajectory_time_list) and \
                            trajectory_time_list[time_segment_index] <= accumulated_time:
                time_list_in_segments[i].append(trajectory_time_list[time_segment_index] -
                                                    accumulated_time + self.time_deque[i][1])
                time_segment_index += 1
        res = []
        # get pose of each distance in different segments
        for i in range(len(self.trajectory_deque)):
            if len(time_list_in_segments[i]) > 0:
                res.extend(self.trajectory_deque[i].get_pose_at_time_list([time_list_in_segments[i][j] + self.time_deque[i][0] for j in range(len(time_list_in_segments[i]))]))
        return res

    def to_planner_result(self, horizon=DEFAULT_TIME_HORIZON,
                          temporal_sampling_number=DEFAULT_SAMPLING_NUMBER,
                          is_extend_trajectory=True):
        time_list = np.linspace(0, horizon, temporal_sampling_number)
        result = self.get_pose_at_time_list(time_list)
        if not is_extend_trajectory:
            return result

        # Extend the trajectory
        index, trajectory_time_offset = self.get_deque_index_at_time(time_list[-1])
        distance = self.trajectory_deque[index].get_dis_at_time(self.time_deque[index][0] + trajectory_time_offset)
        path_len = self.trajectory_deque[index].path.get_length()
        oclog.debug("Extend the trajectory. Current: " + str(distance) + ". Extend to " + str(path_len))
        distance_list = np.linspace(distance, path_len, temporal_sampling_number)
        speed = result[-1].speed
        pose_list = self.trajectory_deque[index].path.get_pose_at_distance_list(distance_list)
        for pose in pose_list:
            pose.speed = speed
            pose.acceleration = 0
            pose.jerk = 0
        result.extend(pose_list[1:])
        oclog.debug("number of pose: " + str(len(result)))
        return result

    def get_deque_index_at_time(self, trajectory_time):
        i = 0
        while trajectory_time > self.time_deque[i][1]:
            trajectory_time -= self.time_deque[i][1]
            i += 1
            if i == len(self.time_deque):
                raise Exception("trajectory time exceeds total time")
        return i, trajectory_time

    def get_time_horizon(self):
        return self.time_horizon

    def transform_trajectory_with_respect_to_pose(self, pose):
        for trajectory in self.trajectory_deque:
            trajectory.transform_trajectory_with_respect_to_pose(pose)


if __name__ == '__main__':
    from data_structures.curve import PolyCurve
    anchors = [Pose([120.209824, 3.247865], -0.052588, 0, 0, 0, 0),
               Pose([189.018901890189, 0.0], 0, 0, 0, 0, 0),
               Pose([212.021202120212, 0.0], 0, 0, 0, 0, 0),
               Pose([235.02350235023499, 0.0], 0, 0, 0, 0, 0),
               Pose([258.02580258025802, 0.0], 0, 0, 0, 0, 0),
               Pose([281.02810281028098, 0.0], 0, 0, 0, 0, 0),
               Pose([304.03040304030401, 0.0], 0, 0, 0, 0, 0),
               ]
    path_segments = []
    length = 0
    for i in range(len(anchors)-1):
        path_segments.append(PolyCurve(anchors[i], anchors[i+1]))
        length += path_segments[-1].get_length()
    print length
    path = BasicPlannerPath(path_segments)
    poses = path.get_pose_at_distance_list(np.linspace(0, path.get_length(), 50))
    a = 0

    trajectory = BasicPlannerTrajectory(path, BasicPlannerSpeedProfile(10, 0, 18.4))
    trajectory.plot(horizon=18.4, temporal_sampling_number=3000)
    trajectory.transform_trajectory_with_respect_to_pose(trajectory.get_pose_at_time(0))
    trajectory.plot(horizon=18.4, temporal_sampling_number=3000)
