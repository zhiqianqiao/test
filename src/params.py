import numpy as np
import ConfigParser

__author__ = 'xhou'


class Param:
    def __init__(self, config_file):
        cfg = ConfigParser.ConfigParser()
        cfg.read(config_file)
        for cur_sec in cfg.sections():
            for cur_param in cfg.items(cur_sec):
                setattr(self, cur_param[0], eval(cur_param[1]))