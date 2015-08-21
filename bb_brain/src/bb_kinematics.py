#! /usr/bin/env python


# BigBug body movement controller


# listens to /bb_hex/body_mode topic for special body commands:
# mode=initial_position -- place body on the ground, raise legs slightly
# mode=storage_position -- place body on the ground, fold legs upwards


# listens to /bb_hex/twist topic for immediate body movement speeds (standard Twist message)

# listens to /bb_hex/stance topic for target clearance, roll and pitch (bb_msgs.msg.Stance message)



# publishes transforms for thorax and limbs

# publishes target servo angles to /bb_hex/servo_targets topic (bb_msgs.msg.ServoTargets message)


import rospy


def run():
    while not rospy.is_shutdown():
        pass


if __name__ == '__main__':
    rospy.init_node('bb_kinematics', anonymous=True)
    try:
        run()
    except rospy.ROSInterruptException:
        pass