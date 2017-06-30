from params import Param
from state_machine import *
__author__ = 'xhou'


def main():
    p = Param('p.cfg')
    test1 = Planner(p)
    test1.update(0, {})


if __name__ == '__main__':
    main()