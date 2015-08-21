#! /usr/bin/env python


# BigBug body movement controller


# subscribes to /bb_hex/body_mode topic for special body commands:
# mode=initial_position -- place body on the ground, raise legs slightly
# mode=storage_position -- place body on the ground, fold legs upwards


# subscribes to /bb_hex/target_twist topic for immediate body movement speeds (standard Twist message)

# subscribes to /bb_hex/target_stance topic for target clearance, roll and pitch (bb_msgs.msg.Stance message)

# subscribes to /bb_hex/servo_current topic for current servo angles and loads (bb_msgs.msg.ServoCurrent message)



# publishes transforms for thorax and limbs

# publishes target servo angles to /bb_hex/servo_targets topic (bb_msgs.msg.ServoTargets message)

# publishes odometry to /bb_hex/odometry topic (standard Odometry message)



import rospy


# parameters









def run():
    while not rospy.is_shutdown():
        pass


if __name__ == '__main__':
    rospy.init_node('bb_kinematics', anonymous=True)
    try:
        run()
    except rospy.ROSInterruptException:
        pass