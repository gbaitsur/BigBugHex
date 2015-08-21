#! /usr/bin/env python


# control panel for BigBug

# subscribes to /bb_hex/voltage topic for current voltage (standard Float32 message)

# subscribes to /bb_hex/odometry topic for actual pose (standard Odometry message)


# publishes immediate body movement speeds to /bb_hex/target_twist topic (standard Twist message)

# publishes target clearance, roll and pitch to /bb_hex/target_stance topic (bb_msgs.msg.Stance message)