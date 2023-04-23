#!/usr/bin/env python3

import rospy
from omnibot_core_py_lib import OMNIBOT_CORE


def main(args=None):
    rospy.init_node('/* node_name */')
    core = OMNIBOT_CORE()
    
    rospy.spin()


if __name__ == '__main__':
    main()
