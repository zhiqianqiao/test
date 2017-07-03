import numpy as np
from math import *



IDEAL_SPEED = 30
SLOW_SPEED = 3
DEFAULT_MAX_ACCELERATION = 3
MAX_JERK = 2
PLAN_TIME  =0.4


class SpeedProfileACC:


    def __init__(self, acceleration_range, sampling_number, time_horizon, plan_time,ideal_speed):
    # calibration_flag is for the case front car is still far away, use damped cotroller to catch,aka catch up slowly

        # acc pid
        self.prev_diff = 0
        self.i_term = 0
        self.prev_target = None
        self.cali_flag = False
        self.ideal_speed = ideal_speed
        self.sampling_time = time_horizon/sampling_number
        self.sampling_number = sampling_number
        self.min_acceleration = acceleration_range[0]
        self.max_acceleration = acceleration_range[1]
        self.plan_time = plan_time

    def get_pid_residual(self):
        return self.prev_diff,self.i_term,self.cali_flag

    def set_pid_residual(self,previous_d_term,previous_i_term,calibration_flag):
        # acc pid
        self.prev_diff = previous_d_term
        self.i_term = previous_i_term
        self.cali_flag = calibration_flag

    def acc_pid(self,ego_speedy, front_speed, front_dist):

        ideal_distance = self.ideal_dist(front_speed)
        rela_v = ego_speedy - front_speed
        diff = front_dist - ideal_distance - rela_v * self.plan_time * 1

        # kd = 0.2
        # ki = 1

        kp = 3
        kd = 0
        ki = 0

        if not self.cali_flag:

            if diff < 0:
                kp = 5
                kd = 0
                ki = 0
            elif diff > 1:
                kd = 5
                ki = 0
                diff = max(1,diff - rela_v**2/self.max_acceleration)
            else:
                self.cali_flag = True
                self.i_term = 0
                self.prev_diff = 0


        if diff > 3:
            diff = max(3, diff - rela_v ** 2 / self.max_acceleration)
            target = min(self.ideal_speed, front_speed + diff*0.14+3)


        else:
            d_term = (diff - self.prev_diff) / self.plan_time

            self.i_term = self.i_term + diff * self.plan_time
            if np.sign(self.prev_diff) != np.sign(diff):
                self.i_term = 0
            target = min(max(self.cost_function(diff)*kp + d_term * kd + self.i_term * ki + front_speed, 0.0), self.ideal_speed)

        if target < 0.1:
            target = 0.0


        self.prev_diff = diff
        return target


    def cost_function(self,diff):
        if diff >= 0:
            return diff ** 2
        else:
            return -diff ** 2

    def ideal_dist(self,front_car_speed):
        # return 0
        return front_car_speed * 0.5


    def speed_profile_generation(self,front_dis,front_speed, ego_speed,prev_acceleration,previous_d_term,previous_i_term,
                                 calibration_flag,emergency_flag = False,ideal_speed = IDEAL_SPEED):
        """
        generate speed profiles. 
        :return:
        """
        self.ideal_speed = ideal_speed

        # if emergency_flag:
        #     self.max_acceleration = 7
        # else:
        #     self.max_acceleration = DEFAULT_MAX_ACCELERATION

        speeds_list = []
        self.set_pid_residual( previous_d_term, previous_i_term,calibration_flag)
        target_speed = self.acc_pid(ego_speed,front_speed,front_dis)

        # TODO: jerk control over acceleration

        temp_speed = ego_speed


        for i in xrange(self.sampling_number):
            if abs(temp_speed-target_speed)>(self.max_acceleration * self.sampling_time):
                temp_speed = temp_speed + np.sign(target_speed - temp_speed) * self.max_acceleration * self.sampling_time
            else:
                temp_speed = target_speed

            # temp_speed = target_speed

            speeds_list.append(temp_speed)


        # for i in xrange(self.sampling_number):
        #     if abs(temp_speed-target_speed)>(self.max_acceleration * self.sampling_time):
        #         temp_speed = temp_speed + np.sign(target_speed - temp_speed) * self.max_acceleration * self.sampling_time
        #     else:
        #         temp_speed = target_speed
        #
        #     # temp_speed = target_speed
        #
        #     speeds_list.append(temp_speed)






        return speeds_list,self.prev_diff,self.i_term,self.cali_flag






