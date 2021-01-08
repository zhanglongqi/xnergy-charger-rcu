#!/usr/bin/env python3

import rospy
from xnergy_charger_rcu.xnergy_ros_wrapper import XnergyChargerROSWrapper
from xnergy_charger_rcu.xnergy_charger_action_server import ChargeActionServer

if __name__ == "__main__":

    rospy.init_node("xnergy_charger_rcu")

    # Initialize XnergyChargerROSWrapper
    xnergy_charger = XnergyChargerROSWrapper()
    rospy.loginfo("Ready.")

    # Initialize XnergyChargerActionServer
    xnergy_action_server = ChargeActionServer(
        rospy.get_name(), "/charge", xnergy_charger)

    # Get Xnergy Charger Status abd publish to ROS topics every 1 hz
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        xnergy_charger.status_update()
        rate.sleep()
