from params import Param
from nav_planner import *
__author__ = 'xhou'


def main():
    p = Param('config/p.cfg')
    test1 = Planner(p)
    test1.update(0, {})


if __name__ == '__main__':
    main()