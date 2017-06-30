from data_structures.planner import Planner

from planner_components.setting_loader import PlannerSetting
from planner_components.process_perception import ProcessPerception
from planner_components.speed_profile_acc import SpeedProfileACC
from planner_components.trajectory_generation import TrajGenerator

from utilities.map_util import PlannerMapUtil
from tsmap import *

#debug
import debugging_tools.octopus_logger as oclog
import rospy
import time


class NPCPlanner1(Planner):
    def __init__(self,map_info,config_path ='../config/toy_config'):
        # read map
        # hdmap = HDMap(map_file_path)
        # map_info = MapInfo(hdmap)
        # # if map_info.lanes.lane_size() == 0:
        # #     oclog.warn('a map must be given. input does not have lanes')
        # #     raise Exception('a map must be given. input does not have lanes')

        self.map_info = map_info

        with open('/mnt/truenas/scratch/data/map/hdmap/data/MAP_new/torrey.hdmap', 'rb') as f:
            submap = f.read()
        self.temp_map = PlannerMapUtil(submap)


        # load config
        self.config = PlannerSetting(config_path)

        self.process = ProcessPerception(map_info,self.temp_map,self.config.ideal_speed,self.config.acceleration_limit,self.config.plan_time)
        self.speed_profile = SpeedProfileACC(self.config.acceleration_limit,self.config.total_sampling_number,self.config.time_horizon,self.config.plan_time)
        self.traj_profile = TrajGenerator(self.temp_map.tsmap)

        self.action = 'acc'
        self.prev_target = -1 # acc target(lane)

        # acc pid
        self.prev_diff = 0
        self.i_term = 0
        self.cali_flag = False


    def plan(self,perception_info):

        rospy.logerr('lets go babe')
        start = time.time()
        self.action,ego_pose,front_dis,front_speed,ego_speed,self.cali_flag = self.process.perception_to_road_coord(perception_info,self.prev_target,self.cali_flag)

        rospy.logerr('test: state->'+str(self.action))


        #TODO
        prev_acceleration = 0
        emergency_flag = False

        speeds_list, self.prev_diff, self.i_term, self.cali_flag = \
            self.speed_profile.speed_profile_generation(front_dis,front_speed,ego_speed,prev_acceleration,
                                                        self.prev_diff,self.i_term,self.cali_flag,emergency_flag,self.config.ideal_speed)

        # rospy.logerr('speed_list->'+str(speeds_list)+'\n ego_pose->'+str(ego_pose)+' sampling->'+str(self.config.total_sampling_number))

        self.action = 'acc'

        trajs, status = self.traj_profile.generate(self.action, speeds_list, perception_info.time_stamp, ego_pose, self.config.total_sampling_number)

        planner_result = {'perceptionTimeStamp': perception_info.time_stamp, 'trajectory': trajs}

        rospy.logerr('planner_result->'+str(planner_result))
        rospy.logerr('runtime->'+str(time.time()-start))


        return planner_result









