#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from inspire_sdkpy import inspire_hand_defaut, inspire_dds

def main():
    rospy.init_node('finger_control_node')

    rospy.loginfo("Initializing Cyclone DDS...")
    ChannelFactoryInitialize(0)
    rospy.loginfo("Cyclone DDS initialized.")

    pub_r = ChannelPublisher("rt/inspire_hand/ctrl/r", inspire_dds.inspire_hand_ctrl)
    pub_r.Init()
    rospy.loginfo("DDS publisher initialized for right hand.")

    cmd = inspire_hand_defaut.get_inspire_hand_ctrl()
    cnd = 0
    seq_idx = 0

    # Define motion sequence (only one frame for now)
    sequence = [
        [100, 900, 900, 900, 900, 900],  
    ]

    rate = rospy.Rate(10)  # 10 Hz (0.1s)
    rospy.loginfo("Finger movement sequence started.")

    while not rospy.is_shutdown():
        try:
            # Select target finger angles
            target = sequence[seq_idx]

            # Fill DDS command
            cmd.angle_set = target
            cmd.mode = 0b0001

            # Publish
            pub_r.Write(cmd)

            # Advance to next state every 10 loops (~1 sec)
            if (cnd + 1) % 10 == 0:
                seq_idx = (seq_idx + 1) % len(sequence)

            cnd += 1
            rate.sleep()

        except Exception as e:
            rospy.logwarn("Failed to publish DDS message: %s", str(e))

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
