#!/usr/bin/env python3

import rospy
import actionlib
from xnergy_charger_rcu.msg import ChargerState, ChargeAction, ChargeFeedback, ChargeResult
from xnergy_charger_rcu.utils import translate_charge_status


class ChargeActionServer:
	# create messages that are used to publish feedback/result

	def __init__(self, namespace, action_name, rcu_unit):
		"""
        Get control from XnergyROSWrapper and Wait for Action Client
        to send Action Goal to trigger goal_callback()
        """
		self._action_name = namespace + action_name
		self._as = actionlib.SimpleActionServer(self._action_name, ChargeAction, execute_cb=self.goal_callback, auto_start=False)

		# Get handle from the Xnergy Modbus/CANbus Interface
		self._driver = rcu_unit

		if not rospy.is_shutdown():
			self._as.start()

	def goal_callback(self, goal):
		"""
        Get Goal from ActionClient and send the charge enable command to RCU unit,
        after that keep monitoring RCU status and publish Action Feedback.
        It will return the Action Result to Action Client whenever the goal is success or fail.
        """
		# publish info
		if (goal.enable_charge):
			rospy.loginfo("Enabling charging")
		else:
			rospy.loginfo("Disabling charging")

		feedback_msg = ChargeFeedback()

		# Send goal command to Xnergy RCU
		# If there is communication error, Action Server will abort the goal
		result = self._driver.charging_switch(goal.enable_charge)

		goal_failed = False
		if not result:
			goal_failed = True
			fail_msg = "Failed: Failed send RCU command."

		feedback_msg.status = self._driver.rcu_status
		self._as.publish_feedback(feedback_msg)

		# Wait until command is received by Xnergy RCU Unit
		rospy.sleep(rospy.Duration(1))
		# Set Action Server as active
		rate = rospy.Rate(2)
		timer = 0
		errors = set()
		while not rospy.is_shutdown() and not goal_failed:
			feedback_msg.status = self._driver.rcu_status
			self._as.publish_feedback(feedback_msg)
			rcu_state = self._driver.rcu_status.state

			current_errors = set(self._driver.rcu.errors)
			errors = errors | current_errors
			error_code = self._driver.rcu.error_code
			shadow_error_code = self._driver.rcu.shadow_error_code

			# when RCU unit status is in 'stop', 'error', 'debug_mode', the action will be aborted
			if self._driver.comm_interface in ["canbus", "modbus"]:
				rcu_state_name = translate_charge_status(rcu_state)
				if goal.enable_charge:
					# check charging is enabled or not
					if 'handshake' == rcu_state_name:
						goal_failed = False
						rospy.loginfo("RCU is in handshake")
					elif 'charging' == rcu_state_name:
						goal_failed = False
						rospy.loginfo("RCU is in charging")
						break
					else:
						rospy.logerr(f'RCU is in error state {rcu_state} error code {error_code:08X} shadow error code {shadow_error_code:08X}')
						goal_failed = True
						fail_msg = f'start charging failed, charger state: {rcu_state} error code {error_code:08X} shadow error code {shadow_error_code:08X}'
						break
				else:
					# check charging is disabled or not
					if rcu_state_name in ('idle', ):
						goal_failed = False
						break
					elif rcu_state_name in ('stopping', ):
						rospy.loginfo("stopping charging")

					elif rcu_state_name in ['XNERGY_RESERVED', 'error']:
						goal_failed = True
						fail_msg = f'stop charging failed, charger state: {rcu_state} error code {error_code:08X} shadow error code {shadow_error_code:08X}'
						break

			elif self._driver.comm_interface == "gpio":
				goal_failed = False
				break
			# check that preempt has not been requested by the client
			if self._as.is_preempt_requested():
				fail_msg = "Failed: Goal was cancelled."
				rospy.logwarn(fail_msg)
				break
			if timer >= 30:
				fail_msg = "Failed: Action timeout, it took more than 30 seconds."
				rospy.logwarn(fail_msg)
				goal_failed = True
				break
			timer += 1
			rate.sleep()

		# publish final feedback
		feedback_msg.status = self._driver.rcu_status
		self._as.publish_feedback(feedback_msg)

		# publish result
		result_msg = ChargeResult()

		if self._as.is_preempt_requested():
			self._as.set_preempted()
		elif not goal_failed:
			result_msg.success = True
			result_msg.message = "Success"
			rospy.logdebug("ChargeActionServer: Goal success")

			self._as.set_succeeded(result_msg)
		else:
			if (len(errors) > 0):
				fail_msg = fail_msg + ":" + ";".join(errors)
			result_msg.success = False
			result_msg.message = fail_msg
			if (goal.enable_charge):
				rospy.logwarn("ChargeActionServer: Failed to enable charging.")
			else:
				rospy.logwarn("ChargeActionServer: Failed to disable charging.")
			self._as.set_aborted(result_msg)
