#! /usr/bin/env python


# control panel for BigBug

# listens to /bb_hex/voltage topic for current voltage (standard Float32 message)

# listens to /bb_hex/pose topic for actual pose (standard Pose message)


# publishes immediate body movement speeds to /bb_hex/target_twist topic (standard Twist message)

# publishes target clearance, roll and pitch to /bb_hex/target_stance topic (bb_msgs.msg.Stance message)