#!/usr/bin/env python3

import rospy
import signal
import sys

from xnergy_charger_rcu.xnergy_ros_wrapper import XnergyChargerROSWrapper
from xnergy_charger_rcu.xnergy_charger_action_server import ChargeActionServer
from xnergy_charger_rcu.range_check_action_server import RangeCheckActionServer


def signal_handler(sig, frame):
	msg = 'You pressed Ctrl+C, shuting down the node, xnergy_charger_rcu!'
	rospy.logdebug(msg)
	rospy.signal_shutdown(msg)
	sys.exit(0)


if __name__ == "__main__":

	rospy.init_node("xnergy_charger_rcu", log_level=rospy.INFO)

	signal.signal(signal.SIGINT, signal_handler)

	# Initialize XnergyChargerROSWrapper
	xnergy_charger = XnergyChargerROSWrapper()
	rospy.loginfo(f'{rospy.get_name()} is ready.')

	# Initialize XnergyChargerActionServer
	xnergy_action_server = ChargeActionServer(rospy.get_name(), "/charge", xnergy_charger)
	range_check_action_server = RangeCheckActionServer(rospy.get_name(), "/rangecheck", xnergy_charger)

	# Get Xnergy Charger Status abd publish to ROS topics in 1 Hz
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		xnergy_charger.status_update()
		rate.sleep()
