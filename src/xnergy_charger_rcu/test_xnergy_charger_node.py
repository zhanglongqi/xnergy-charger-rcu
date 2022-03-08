#!/usr/bin/env python3

import rospy
import actionlib

import sys
import time
import unittest

from dataclasses import dataclass

from xnergy_charger_rcu.msg import RangeCheckAction, RangeCheckGoal, RangeCheckResult, ChargerState
from xnergy_charger_rcu.srv import DummyComm, DummyCommRequest, DummyCommResponse
PKG = 'xnergy_charger_rcu'
NAME = 'range_check_test'


class RangeCheckTest(unittest.TestCase):
	def __init__(self, *args):
		super(RangeCheckTest, self).__init__(*args)
		rospy.init_node(NAME, log_level=rospy.INFO)
		self.comm_interface = rospy.get_param("~comm_interface", None)

		self.t_start = None

	def setUp(self):
		# warn on /use_sim_time is true
		use_sim_time = rospy.get_param('/use_sim_time', False)
		self.t_start = time.time()
		while not rospy.is_shutdown() and use_sim_time and (rospy.Time.now() == rospy.Time(0)):
			rospy.logwarn_throttle(1, '/use_sim_time is specified and rostime is 0, /clock is published?')
			if time.time() - self.t_start > 10:
				self.fail('Timed out (10s) of /clock publication.')
			# must use time.sleep because /clock isn't yet published, so rospy.sleep hangs.
			time.sleep(0.1)

	def test_range_check_action(self):
		t_timeout_max = 5
		while True:
			t_now = time.time()
			t_elapsed = t_now - self.t_start
			if t_elapsed > t_timeout_max:
				rospy.logwarn(msg=f'Timed out (max {t_timeout_max}s) of waiting range check action server.')
				break
			topics = rospy.get_published_topics()
			range_check_action_server_ready = False
			for t in topics:
				if ['/xnergy_charger_rcu/rangecheck/result', 'xnergy_charger_rcu/RangeCheckActionResult'] in topics:
					range_check_action_server_ready = True
					break
			if range_check_action_server_ready:
				rospy.logdebug('range check action server is ready.')
				break
			time.sleep(1)

		client = actionlib.SimpleActionClient('/xnergy_charger_rcu/rangecheck', RangeCheckAction)

		@dataclass
		class TestPair:
			main_state: int
			expected_result: int
			error_code: int = 0
			range_check_status: int = 0

		test_args = (
			TestPair(main_state=ChargerState.RCU_IDLE,
						range_check_status=RangeCheckResult.IN_RANGE,
						expected_result=(RangeCheckResult.IN_RANGE, RangeCheckResult.OUT_RANGE, RangeCheckResult.FAILED, RangeCheckResult.RANGE_CHECKING_NOT_APPLICABLE)),
			TestPair(main_state=ChargerState.RCU_IDLE,
						range_check_status=RangeCheckResult.OUT_RANGE,
						expected_result=(RangeCheckResult.IN_RANGE, RangeCheckResult.OUT_RANGE, RangeCheckResult.FAILED, RangeCheckResult.RANGE_CHECKING_NOT_APPLICABLE)),
			TestPair(main_state=ChargerState.RCU_IDLE,
						range_check_status=RangeCheckResult.FAILED,
						expected_result=(RangeCheckResult.IN_RANGE, RangeCheckResult.OUT_RANGE, RangeCheckResult.FAILED, RangeCheckResult.RANGE_CHECKING_NOT_APPLICABLE)),
			#
			TestPair(main_state=ChargerState.RCU_CHARGING, expected_result=(RangeCheckResult.RANGE_CHECKING_NOT_APPLICABLE, )),
			#
			TestPair(main_state=ChargerState.RCU_ERROR, error_code=0x12345678, expected_result=(RangeCheckResult.RANGE_CHECKING_NOT_APPLICABLE, )),
			#
			TestPair(main_state=ChargerState.RCU_NOT_CONNECTED, expected_result=(RangeCheckResult.RANGE_CHECKING_NOT_APPLICABLE, )),
			TestPair(main_state=ChargerState.RCU_HANDSHAKE_0, expected_result=(RangeCheckResult.RANGE_CHECKING_NOT_APPLICABLE, )),
			TestPair(main_state=ChargerState.RCU_HANDSHAKE_1, expected_result=(RangeCheckResult.RANGE_CHECKING_NOT_APPLICABLE, )),
			TestPair(main_state=ChargerState.RCU_HANDSHAKE_2, expected_result=(RangeCheckResult.RANGE_CHECKING_NOT_APPLICABLE, )),
			TestPair(main_state=ChargerState.RCU_HANDSHAKE_3, expected_result=(RangeCheckResult.RANGE_CHECKING_NOT_APPLICABLE, )),
			TestPair(main_state=ChargerState.RCU_STOP, expected_result=(RangeCheckResult.RANGE_CHECKING_NOT_APPLICABLE, )),
			TestPair(main_state=ChargerState.RCU_SEC_RESTART_DELAY, expected_result=(RangeCheckResult.RANGE_CHECKING_NOT_APPLICABLE, )),
			TestPair(main_state=ChargerState.RCU_SEC_WAIT_TIE_TO_BOOTLOADER, expected_result=(RangeCheckResult.RANGE_CHECKING_NOT_APPLICABLE, )),
			TestPair(main_state=ChargerState.RCU_SEC_PRE_READY, expected_result=(RangeCheckResult.RANGE_CHECKING_NOT_APPLICABLE, )),
			TestPair(main_state=ChargerState.RCU_SEC_WAIT_RUN_CMD, expected_result=(RangeCheckResult.RANGE_CHECKING_NOT_APPLICABLE, )),
			TestPair(main_state=ChargerState.RCU_SEC_RAMP_DOWN, expected_result=(RangeCheckResult.RANGE_CHECKING_NOT_APPLICABLE, )),
			TestPair(main_state=ChargerState.RCU_SEC_MANUAL_CHARGE, expected_result=(RangeCheckResult.RANGE_CHECKING_NOT_APPLICABLE, )),
			TestPair(main_state=ChargerState.RCU_SEC_DEBUG_READY, expected_result=(RangeCheckResult.RANGE_CHECKING_NOT_APPLICABLE, )),
			TestPair(main_state=ChargerState.RCU_SEC_DEBUG_UNPAIR, expected_result=(RangeCheckResult.RANGE_CHECKING_NOT_APPLICABLE, )),
			TestPair(main_state=ChargerState.RCU_SEC_DEBUG_PAIRED, expected_result=(RangeCheckResult.RANGE_CHECKING_NOT_APPLICABLE, )),
			TestPair(main_state=ChargerState.RCU_SEC_DEBUG_SEC_ENABLE_PRI, expected_result=(RangeCheckResult.RANGE_CHECKING_NOT_APPLICABLE, )),
			TestPair(main_state=ChargerState.RCU_SEC_DEBUG_ERROR, expected_result=(RangeCheckResult.RANGE_CHECKING_NOT_APPLICABLE, )),
		)
		# Waits until the action server has started up and started
		# listening for goals.
		client.wait_for_server()

		# Creates a goal to send to the action server.
		goal = RangeCheckGoal(start_rangecheck=True)

		def feedback_cb(msg):
			# rospy.logdebug(f'feedback_cb() {type(msg)} {msg}')
			pass

		def active_cb():
			# rospy.logdebug('active_cb() active')
			pass

		def done_cb(state, result):
			# rospy.logdebug(f'done_cb() {type(state)} {state}, {type(result)} {result}')
			pass

		for test_arg in test_args:
			if 'dummy' == self.comm_interface:
				response: DummyCommResponse = None
				success = True

				set_dummy_status = rospy.ServiceProxy('/xnergy_charger_rcu/set_status', DummyComm)
				response = set_dummy_status(DummyCommRequest(name='main_state', value=test_arg.main_state))
				success = response.success and success
				response = set_dummy_status(DummyCommRequest(name='error_code', value=test_arg.error_code))
				success = response.success and success
				response = set_dummy_status(DummyCommRequest(name='range_check_status', value=test_arg.range_check_status))
				success = response.success and success

				if not success:
					rospy.logerr('Failed to set dummy status')
					return
			elif self.comm_interface in ('canbus', 'modbus'):
				pass
			else:
				return
			rospy.logdebug(f'Test {test_arg}')
			client.send_goal(
				goal,
				feedback_cb=feedback_cb,
				active_cb=active_cb,
				done_cb=done_cb,
			)
			client.wait_for_result()
			result = client.get_result()
			self.assertIn(result.status, test_arg.expected_result)
			time.sleep(2)
		# # Sends the goal to the action server.
		# client.send_goal(
		# 	goal,
		# 	feedback_cb=feedback_cb,
		# 	active_cb=active_cb,
		# 	done_cb=done_cb,
		# )

		# # Waits for the server to finish performing the action.
		# client.wait_for_result()

		# # Prints out the result of executing the action
		# result = client.get_result()
		# state = client.get_state()
		# state_text = client.get_goal_status_text()
		# rospy.logwarn(f'Result: {result.status}-{result.message}, State: {state}-{state_text}')


if __name__ == '__main__':

	import rostest

	rostest.run(PKG, NAME, RangeCheckTest, sys.argv)
