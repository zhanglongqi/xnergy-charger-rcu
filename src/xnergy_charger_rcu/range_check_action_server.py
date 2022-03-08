#!/usr/bin/env python3

import rospy
import time
import actionlib
from actionlib_msgs.msg import GoalStatus
from xnergy_charger_rcu.msg import RangeCheckAction, RangeCheckFeedback, RangeCheckResult


class RangeCheckActionServer:
	# create messages that are used to publish feedback/result

	def __init__(self, namespace, action_name, rcu_unit):
		"""
        Get control from XnergyROSWrapper and Wait for Action Client
        to send Action Goal to trigger goal_callback()
        """
		self._action_name = namespace + action_name
		self._as = actionlib.SimpleActionServer(self._action_name, RangeCheckAction, execute_cb=self.goal_callback, auto_start=False)

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
		if (goal.start_rangecheck):
			rospy.loginfo("start range check")
		else:
			error_msg = "range check can not be canceled"
			rospy.logwarn(error_msg)
			self._as.set_aborted(text=error_msg)
			return GoalStatus.REJECTED

		ready, message = self._driver.pre_range_check()
		if not ready:
			rospy.logwarn(f'range check rejected, {message}')
			self._as.set_aborted(RangeCheckResult(RangeCheckResult.RANGE_CHECKING_NOT_APPLICABLE, message), text=message)
			return GoalStatus.REJECTED

		feedback_msg = RangeCheckFeedback()
		result_msg = RangeCheckResult()

		# Send goal command to Xnergy RCU
		# If there is communication error, Action Server will abort the goal
		success, status, message = self._driver.request_range_check()

		if not success:
			rospy.logerr(f'sending range check request failed, {message}')
			self._as.set_aborted(RangeCheckResult(status=RangeCheckResult.RANGE_CHECKING_NOT_APPLICABLE, message=message))
			return GoalStatus.LOST

		# Wait until command is received by Xnergy RCU Unit
		rospy.sleep(rospy.Duration(1))

		# Set Action Server as active
		rate = rospy.Rate(2)
		timer = time.time()

		while not rospy.is_shutdown():
			if time.time() - timer > 20:
				result_msg.status = RangeCheckResult.TIMEOUT
				result_msg.message = 'timeout, please retry'
				rospy.logwarn(msg=result_msg.message)
				self._as.set_aborted(result_msg, text=result_msg.message)
				return GoalStatus.ABORTED

			success, status, message = self._driver.read_range_check_status()
			if not success:
				continue

			feedback_msg.status = status
			feedback_msg.message = message
			self._as.publish_feedback(feedback_msg)

			result_msg.status = status
			result_msg.message = message

			# check that preempt has not been requested by the client
			if self._as.is_preempt_requested():
				self._as.preempt_request = False
				rospy.logwarn('range checking can only be done once at a time and can not be canceled.')

			if RangeCheckFeedback.ONGOING == status:
				pass
			elif status in (RangeCheckFeedback.IN_RANGE, RangeCheckFeedback.OUT_RANGE):
				rospy.loginfo(msg=result_msg.message)
				self._as.set_succeeded(result_msg, text=f'range check succeeded {result_msg.message}')
				return GoalStatus.SUCCEEDED
			elif status in (RangeCheckFeedback.FAILED, RangeCheckFeedback.RANGE_CHECKING_NOT_APPLICABLE):
				rospy.logwarn(msg=result_msg.message)
				self._as.set_aborted(result_msg, text=result_msg.message)
				return GoalStatus.ABORTED

			rate.sleep()
