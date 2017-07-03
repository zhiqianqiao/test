import numpy as np
from math import *
from numpy.linalg import inv
# from utilities.map_util import PlannerMapUtil
from tsmap import *

import os
import sys
import rospy

MAX_PERCEPTION_DIST = 100
DEFAULT_SAFE_DISTANCE = 6
EMERGENCY_DIST = 3


LEFT_INDEX = 1
FRONT_LEFT_INDEX = 0
REAR_LEFT_INDEX = 2
FRONT_INDEX = 3
CENTER_INDEX = 4
FRONT_RIGHT_INDEX = 6
RIGHT_INDEX = 7
REAR_RIGHT_INDEX = 8

TEMPX_INDEX = 0
TEMPY_INDEX = 1
TEMPH_INDEX = 3
SPEEDX_INDEX = 5
SPEEDY_INDEX = 6
LANE_INDEX = 7
PLAN_TIME = 0.4

class CarGrid:
    def __init__(self, map_info, ideal_speed, acceleration_limit, plan_time):
        self.map_info = map_info
        self.perception_info = None
        self.perception_in_grid = None
        self.merge_index = 0
        self.prev_target = -1
        self.front_dis = 100
        self.front_speed = 0
        self.target_index = -1
        self.cali_flag = False
        self.current_lane = None

        self.prev_lane_heading = None

        self.plan_time = plan_time
        self.ideal_speed = ideal_speed
        self.max_deceleration = acceleration_limit[0]
        self.max_acceleration = acceleration_limit[1]

        # sys.path.append('./planning/src')

        AD = os.path.dirname(os.path.abspath(__file__))
        rospy.logerr('directtory->' + str(AD))

        with open(
                '/home/ganyq/Documents/June/mp-simworld/src/octopus-planner/src/motion_planner/library/mp-basic-planner/planner/torrey.hdmap',
                'rb') as f:
            submap = f.read()
        # map = TSMap.deserialize(submap)
        self.temp_map = PlannerMapUtil(submap)

    def find_grid_index(self, lane, tempx, tempy):

        grid_size = 2.5

        if lane == None:
            if (tempx < -2.5 and tempx > -7.5):  # left lane and left left lane
                if tempy > grid_size:  # one and half of the car length
                    return 0
                elif tempy < -grid_size:
                    return 2
                else:
                    return 1

            elif (tempx > 1.5 and tempx < 7.5):  # right and right right lane
                if tempy > grid_size:  # one and half of the car length
                    return 6
                elif tempy < -grid_size:
                    return 8
                else:
                    return 7

            elif (tempx <= 1.5 and tempx > -1.5):  # mid lane
                if tempy > grid_size:  # one and half of the car length
                    return 3
                elif tempy < -grid_size:
                    return 5
                else:
                    return 4
        else:
            left_lane_num = len(self.left_lanes)
            right_lane_num = len(self.right_lanes)

            if lane == self.current_lane:  # mid lane
                if tempy > grid_size:  # one and half of the car length
                    return 3
                elif tempy < -grid_size:
                    return 5
                else:
                    return 4

            # on the left

            if left_lane_num >= 2:
                if lane in self.left_lanes[-2:]:
                    if tempy > grid_size:  # one and half of the car length
                        return 0
                    elif tempy < -grid_size:
                        return 2
                    else:
                        return 1
            if left_lane_num == 1:
                if lane in self.left_lanes[-1:]:
                    if tempy > grid_size:  # one and half of the car length
                        return 0
                    elif tempy < -grid_size:
                        return 2
                    else:
                        return 1

            # on the rigth
            if right_lane_num >= 2:
                if lane in self.right_lanes[-2:]:
                    if tempy > grid_size:  # one and half of the car length
                        return 6
                    elif tempy < -grid_size:
                        return 8
                    else:
                        return 7
            if right_lane_num == 1:

                if lane in self.right_lanes[-1:]:
                    if tempy > grid_size:  # one and half of the car length
                        return 6
                    elif tempy < -grid_size:
                        return 8
                    else:
                        return 7

        return -1

    def lane_numbering(self):

        right = self.current_lane.right_lane
        right_list = []
        while right != None:
            right_list.append(right)
            right = right.right_lane

        left = self.current_lane.left_lane
        left_list = []
        while left != None:
            left_list.append(left)
            left = left.left_lane

        left_list = left_list[::-1]
        self.left_lanes = left_list
        self.right_lanes = right_list

        self.lane_list = left_list + [self.current_lane] + right_list

    def rotation2local(self, ego_x, ego_y, ego_heading, tempx, tempy, temp_heading):
        theta = ego_heading - pi / 2
        tranx = ego_x
        trany = ego_y

        rot = [[cos(theta), -sin(theta), tranx], [sin(theta), cos(theta), trany], [0, 0, 1]]
        rot = np.asmatrix(rot)

        theta = temp_heading - pi / 2
        tranx = tempx
        trany = tempy
        rot2 = [[cos(theta), -sin(theta), tranx], [sin(theta), cos(theta), trany], [0, 0, 1]]
        rot2 = np.asmatrix(rot2)

        tempv = [0, 0, 1]
        temp = np.dot(inv(rot), rot2)
        temp = np.asarray(np.dot(temp, tempv))
        return temp[0][0], temp[0][1], ego_heading - temp_heading

    def road_vector(self, x, y):  # Find road direction



        temp_lane = self.temp_map.get_lane(x, y)

        p = Point3d(x, y)
        p_now = self.temp_map.tsmap.get_ref_pt(p)
        p_next, mov_dis = temp_lane.move_forward(p_now, 0.5)

        road_dir = [p_next.x - p_now.x, p_next.y - p_now.y]

        return road_dir

    def perception_to_road_coord(self, perception_info, prev_target, cali_flag):
        """
        pre-process perception data into 3 by 3 grids
        determine if there is objects in the neighboring grid, return the nearest object in that grid
        input:
        output: a tuple in size of 8*5+8( eight (x,y,w,h,vx,vy,lane) + 8 dimension for if road exists for each grid)

        Horizon grid size: 2 lanes(6m)|lane(3m)|2 lanes(6m)
        Vertical grid size: +30m|3 car lengths(15m)|-30m

        Grid index:
        0|3|6
        -----
        1|4|7
        -----
        2|5|8

        Warning:   1. assume ego car ID is vehicle 0 now, on the real car, ID could change
                    2. assume the same time stamp exists for the perceptions and planning, in the future, interpolation is needed if time stamp appears different


        """
        # get ego information


        self.perception_info = perception_info
        self.prev_target = prev_target
        self.cali_flag = cali_flag

        position = self.perception_info.ego_info['pose']['position']
        ego_x = position[0]
        ego_y = position[1]

        self.current_lane = self.temp_map.get_lane(ego_x, ego_y)

        self.lane_numbering()

        self.ego_speed = self.perception_info.ego_info['pose']['speed']
        ego_info = self.perception_info.ego_info['pose']

        # self.ego_pose = Pose(ego_info['position'], ego_info['heading'], ego_info['curvature'], ego_info['speed'],
        #                      ego_info['acceleration'], ego_info['jerk'])


        road_dir = self.road_vector(ego_x, ego_y)
        if self.current_lane == None or np.linalg.norm(road_dir) <= 0.01:
            lane_heading = self.prev_lane_heading
        else:
            lane_heading = np.arctan2(road_dir[1], road_dir[0])

        self.prev_lane_heading = lane_heading

        self.ego_heading = lane_heading - ego_info['heading']

        self.ego_speedx = self.ego_speed * sin(self.ego_heading)
        self.ego_speedy = self.ego_speed * cos(self.ego_heading)

        self.safe_distance = max(self.ego_speedy ** 2 / 2.0 / np.absolute(self.max_deceleration), DEFAULT_SAFE_DISTANCE)

        # initial objects in grids
        grid_dist_list = np.ones(9) * 999999999.0

        grid_num = 9
        grid_dim = 8
        grid_precption = np.zeros(grid_num * grid_dim).reshape(grid_num, grid_dim)

        # grid_precption = np.matlib.repmat(np.array(0.0), 9, 7)
        # grid default info:
        # each row/grid: [tempx, tempy, temp_heading, temp_h, temp_w, speedx, speedy, lane]


        grid_precption[0][0], grid_precption[0][1] = -MAX_PERCEPTION_DIST, MAX_PERCEPTION_DIST
        grid_precption[1][0], grid_precption[1][1] = -MAX_PERCEPTION_DIST, 0
        grid_precption[2][0], grid_precption[2][1] = -MAX_PERCEPTION_DIST, -MAX_PERCEPTION_DIST

        grid_precption[3][0], grid_precption[3][1] = 0.0, MAX_PERCEPTION_DIST
        grid_precption[5][0], grid_precption[5][1] = 0.0, -MAX_PERCEPTION_DIST

        grid_precption[6][0], grid_precption[6][1] = MAX_PERCEPTION_DIST, MAX_PERCEPTION_DIST
        grid_precption[7][0], grid_precption[7][1] = MAX_PERCEPTION_DIST, 0
        grid_precption[8][0], grid_precption[8][1] = MAX_PERCEPTION_DIST, -MAX_PERCEPTION_DIST

        # process dynamic objects
        for car in self.perception_info.dynamic_info:
            # ignore the planner car
            # if car['id'] == EGO_CAR_ID:
            #     continue
            position = car['position']
            tempx = position[0]
            tempy = position[1]

            temp_lane = self.temp_map.get_lane(tempx, tempy)

            temp_heading = car['heading']

            # calculate the lane heading
            road_dir = self.road_vector(tempx, tempy)
            lane_heading = np.arctan2(road_dir[1], road_dir[0])

            tempx, tempy, temp_heading = self.rotation2local(ego_x, ego_y, lane_heading, tempx, tempy, temp_heading)
            # calculate the closest object in each grid
            grid_index = self.find_grid_index(temp_lane, tempx, tempy)
            if grid_index == -1: continue

            temp_dist = np.linalg.norm([tempx, tempy])

            if grid_dist_list[grid_index] > temp_dist:
                grid_dist_list[grid_index] = temp_dist
                grid_precption[grid_index][0], grid_precption[grid_index][1], grid_precption[grid_index][
                    2] = tempx, tempy, temp_heading
                grid_precption[grid_index][3], grid_precption[grid_index][
                    4] = 4.9, 1.9  # assume the moving object is car now, get geometric data if possible in the future
                grid_precption[grid_index][5], grid_precption[grid_index][6] = car['speed'] * sin(temp_heading), car[
                    'speed'] * cos(temp_heading)
                grid_precption[grid_index][7] = temp_lane.id

        # process static objects
        for obstacle in self.perception_info.static_info:
            position = obstacle['position']

            tempx = position[0]
            tempy = position[1]

            temp_lane = self.temp_map.get_lane(tempx, tempy)
            # temp_heading = obstacle['heading']

            # calculate the lane heading
            road_dir = self.road_vector(tempx, tempy)
            lane_heading = np.arctan2(road_dir[1], road_dir[0])

            tempx, tempy, temp_heading = self.rotation2local(ego_x, ego_y, lane_heading, tempx, tempy, lane_heading)
            # calculate the closest object in each grid
            grid_index = self.find_grid_index(temp_lane, tempx, tempy)
            if grid_index == -1: continue

            temp_dist = np.linalg.norm([tempx, tempy])

            if grid_dist_list[grid_index] > temp_dist:
                grid_dist_list[grid_index] = temp_dist
                grid_precption[grid_index][0], grid_precption[grid_index][1], grid_precption[grid_index][
                    2] = tempx, tempy, temp_heading
                grid_precption[grid_index][3], grid_precption[grid_index][
                    4] = 1, 1  # assume static object is in size of 1m x 1m
                grid_precption[grid_index][5], grid_precption[grid_index][6] = 0, 0
                grid_precption[grid_index][7] = temp_lane.id

        for ii in xrange(9):
            grid_precption[ii, TEMPY_INDEX] = grid_precption[ii, TEMPY_INDEX] - grid_precption[ii, TEMPH_INDEX] * 0.5

        self.perception_in_grid = grid_precption

        self.merge_check()
        self.finite_states()
        self.acc()

        return self.state, self.front_dis, self.front_speed, self.ego_speed, self.cali_flag

    def merge_check_helper(self, left_flag, emergency_flag=False):

        # helper function for merge check
        # check if left or right cars has potential to hit
        # in the check horizon if any cars are coming into a box



        normal_horizon = 1
        emergency_horizon = 0.6
        check_interval = 0.05

        comfort_dist_x = 2
        if emergency_flag:
            comfort_dist_y = 5
            check_horizon = emergency_horizon
        else:
            comfort_dist_y = 8
            check_horizon = normal_horizon

        check_flag = True
        if left_flag:
            # check left
            if len(self.left_lanes) == 0:
                return False

            for ii in xrange(6):
                if self.perception_in_grid[ii, TEMPH_INDEX] != 0:
                    temp_x = self.perception_in_grid[ii, TEMPX_INDEX]
                    temp_y = self.perception_in_grid[ii, TEMPY_INDEX]

                    if temp_x < 0 and temp_x > -5 \
                            and temp_y < comfort_dist_y and temp_y > -comfort_dist_y:
                        check_flag = False
                    else:

                        relative_speedx = self.ego_speedx - self.perception_in_grid[ii, SPEEDX_INDEX]
                        relative_speedy = self.ego_speedy - self.perception_in_grid[ii, SPEEDY_INDEX]

                        for ii in xrange(int(check_horizon / check_interval)):
                            temp_x = temp_x - check_interval * relative_speedx
                            temp_y = temp_y - check_interval * relative_speedy

                            if temp_x < 0 and temp_x > -5 \
                                    and temp_y < comfort_dist_y and temp_y > -comfort_dist_y:
                                check_flag = False


        else:
            # check right
            if len(self.right_lanes) == 0:
                return False
            for ii in xrange(3, 9):
                if self.perception_in_grid[ii, TEMPH_INDEX] != 0:
                    temp_x = self.perception_in_grid[ii, TEMPX_INDEX]
                    temp_y = self.perception_in_grid[ii, TEMPY_INDEX]

                    if temp_x > 0 and temp_x < 5 \
                            and temp_y < comfort_dist_y and temp_y > -comfort_dist_y:

                        check_flag = False

                    else:
                        relative_speedx = self.ego_speedx - self.perception_in_grid[ii, SPEEDX_INDEX]
                        relative_speedy = self.ego_speedy - self.perception_in_grid[ii, SPEEDY_INDEX]

                        # rospy.logerr('relative speedx->'+str(relative_speedx)+' speed y->'+str(relative_speedy))

                        for jj in xrange(int(check_horizon / check_interval)):
                            temp_x = temp_x - check_interval * relative_speedx
                            temp_y = temp_y - check_interval * relative_speedy
                            # rospy.logerr('relative_speedx'+str(relative_speedx)+'relative_speedy'+str(relative_speedy)+'tempx->'+str(temp_x)+' tempy->'+str(temp_y))
                            if temp_x > 0 and temp_x < 5 \
                                    and temp_y < comfort_dist_y and temp_y > -comfort_dist_y:
                                check_flag = False
        # rospy.logerr('check_flag->'+str(check_flag))
        return check_flag

    def merge_check(self):
        # check if its okay to merge
        # output:    0: not okay
        #            -1: okay to merge left in normal case
        #            -2: okay to merge left in emergency case
        #             1: okay to merge right in normal case
        #             2: okay to merge right in emergency case



        front_rel_speed = self.ego_speedy - self.perception_in_grid[
            FRONT_INDEX, SPEEDY_INDEX]  # >0 has potential to crash

        merge_left_flag = self.merge_check_helper(True, False)
        merge_left_urgent = self.merge_check_helper(True, True)

        merge_right_flag = self.merge_check_helper(False, False)
        merge_right_urgent = self.merge_check_helper(False, True)

        # rospy.logerr('merge_states->' + str(merge_left_flag) + ' ' + str(merge_left_urgent) + ' ' + str(
        #     merge_right_flag) + ' ' + str(merge_right_urgent))

        self.merge_index = 0
        # front car is not too close
        if (self.perception_in_grid[FRONT_INDEX, TEMPH_INDEX] != 0 and self.perception_in_grid[
            FRONT_INDEX, 1] > front_rel_speed * self.plan_time) or self.perception_in_grid[
            FRONT_INDEX, TEMPH_INDEX] == 0:

            if merge_left_flag:
                self.merge_index = -1
                return

            if merge_right_flag:
                self.merge_index = 1
                return

            if merge_right_urgent:
                self.merge_index = 2
                return

            if merge_left_urgent:
                self.merge_index = -2
                return

    def finite_states(self):

        if self.perception_in_grid[FRONT_INDEX, TEMPH_INDEX] == 0:
            self.state = 'acc'
            return
        if self.perception_in_grid[FRONT_INDEX, TEMPH_INDEX] != 0:
            if self.perception_in_grid[FRONT_INDEX, SPEEDY_INDEX] > self.ideal_speed * 0.5 or \
                            self.perception_in_grid[FRONT_INDEX, TEMPY_INDEX] > 50 + (
                                self.ego_speedy - self.perception_in_grid[FRONT_INDEX, SPEEDY_INDEX]) * self.plan_time:
                self.state = 'acc'
                return

            else:
                if (self.perception_in_grid[FRONT_LEFT_INDEX, TEMPH_INDEX] == 0
                    or (self.perception_in_grid[FRONT_LEFT_INDEX, TEMPH_INDEX] != 0 and self.perception_in_grid[
                        FRONT_LEFT_INDEX, SPEEDY_INDEX] > self.ego_speed * 1.5)) \
                        and self.merge_index == -1:
                    # if self.current_lane == self.prev_target_lane:
                    #     self.state = 'acc'
                    # else:
                    self.state = 'merge_left'
                    return
                elif (self.perception_in_grid[FRONT_RIGHT_INDEX, TEMPH_INDEX] == 0
                      or (self.perception_in_grid[FRONT_RIGHT_INDEX, TEMPH_INDEX] != 0 and self.perception_in_grid[
                        FRONT_RIGHT_INDEX, SPEEDY_INDEX] > self.ego_speed * 1.5)) \
                        and self.merge_index == 1:
                    # if self.current_lane == self.prev_target_lane:
                    #     self.state = 'acc'
                    # else:
                    self.state = 'merge_right'

                    return

                if self.perception_in_grid[FRONT_INDEX, TEMPY_INDEX] < EMERGENCY_DIST:
                    if self.merge_index == 2:
                        self.state = 'sharp_merge_right'
                        return
                    if self.merge_index == -2:
                        self.state = 'sharp_merge_left'
                        return

        self.state = 'acc'

    def acc(self):
        """
        To keep distance with the front object at relative lane
        :param relative_lane: -1 means left and 1 means right
        :return:

        """
        # very close, then keep speed if front car is accelerating, decelerate if front car is decelerating

        front_left_dist = self.perception_in_grid[FRONT_LEFT_INDEX, TEMPY_INDEX]
        front_right_dist = self.perception_in_grid[FRONT_RIGHT_INDEX, TEMPY_INDEX]
        front_dis = self.perception_in_grid[FRONT_INDEX, TEMPY_INDEX]

        target_index = FRONT_INDEX

        # a object too close
        if (self.perception_in_grid[CENTER_INDEX, TEMPH_INDEX] != 0):
            target_speed = 0
            return target_speed

        # a object in the front left and moving to the middle
        if ((self.perception_in_grid[FRONT_LEFT_INDEX, TEMPH_INDEX] != 0)
            and (self.perception_in_grid[FRONT_LEFT_INDEX, SPEEDX_INDEX]) >= (
                        np.absolute(self.perception_in_grid[FRONT_LEFT_INDEX, TEMPX_INDEX]) - 1.5) / PLAN_TIME):
            if front_dis > front_left_dist:
                target_index = FRONT_LEFT_INDEX

        # a object in the front right and moving to the middle
        if ((self.perception_in_grid[FRONT_RIGHT_INDEX, TEMPH_INDEX] != 0)
            and (-self.perception_in_grid[FRONT_RIGHT_INDEX, SPEEDX_INDEX]) >= (
                        np.absolute(self.perception_in_grid[FRONT_RIGHT_INDEX, TEMPX_INDEX]) - 1.5) / PLAN_TIME):
            if front_dis > front_right_dist:
                target_index = FRONT_RIGHT_INDEX

        # merge
        # TODO check it mate
        if self.state == 'merge_right' or self.state == 'sharp_merge_right':
            if self.perception_in_grid[FRONT_RIGHT_INDEX, TEMPH_INDEX] == 0:
                target_index = FRONT_INDEX
            elif self.perception_in_grid[FRONT_RIGHT_INDEX, TEMPY_INDEX] / (
                self.ego_speedy - self.perception_in_grid[FRONT_RIGHT_INDEX, SPEEDY_INDEX]) < \
                            self.perception_in_grid[FRONT_RIGHT_INDEX, TEMPY_INDEX] / (
                        self.ego_speedy - self.perception_in_grid[FRONT_INDEX, SPEEDY_INDEX]):
                target_index = FRONT_RIGHT_INDEX
            else:
                target_index = FRONT_INDEX


        elif self.state == 'merge_left' or self.state == 'sharp_merge_left':
            if self.perception_in_grid[FRONT_LEFT_INDEX, TEMPH_INDEX] == 0:
                target_index = FRONT_INDEX
            elif self.perception_in_grid[FRONT_LEFT_INDEX, TEMPY_INDEX] / (
                self.ego_speedy - self.perception_in_grid[FRONT_LEFT_INDEX, SPEEDY_INDEX]) < \
                            self.perception_in_grid[FRONT_INDEX, TEMPY_INDEX] / (
                        self.ego_speedy - self.perception_in_grid[FRONT_INDEX, SPEEDY_INDEX]):
                target_index = FRONT_LEFT_INDEX
            else:
                target_index = FRONT_INDEX

        # rospy.logerr('targe_index->' + str(target_index))
        front_dis = self.perception_in_grid[target_index, TEMPY_INDEX]
        front_speed = self.perception_in_grid[target_index, SPEEDY_INDEX]

        if self.prev_target != target_index:
            self.cali_flag = False

        self.front_dis = front_dis
        self.front_speed = front_speed
        self.target_index = target_index
