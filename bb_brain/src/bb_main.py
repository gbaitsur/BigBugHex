#! /usr/bin/env python

# main BigBug routine

import rospy


def run():
    while not rospy.is_shutdown():
        pass


if __name__ == '__main__':
    rospy.init_node('bb_main', anonymous=True)
    try:
        run()
    except rospy.ROSInterruptException:
        pass
