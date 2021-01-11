#!/usr/bin/env python3

import rospy
import actionlib
from xnergy_charger_rcu.msg import ChargerState, ChargeAction, ChargeFeedback, ChargeResult


class ChargeActionServer:
    # create messages that are used to publish feedback/result

    def __init__(self, namespace, action_name, rcu_unit):
        """
        Get control from XnergyROSWrapper and Wait for Action Client 
        to send Action Goal to trigger goal_callback()  
        """
        self._action_name = namespace + action_name
        self._as = actionlib.SimpleActionServer(
            self._action_name, ChargeAction, execute_cb=self.goal_callback, auto_start=False)

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
        self._driver.rcu.reset_errors()
        # publish info
        if(goal.enable_charge):
            rospy.loginfo("Enabling charging")
        else:
            rospy.loginfo("Disabling charging")

        feedback_msg = ChargeFeedback()

        # Send goal command to Xnergy RCU
        # If there is communication error, Action Server will abort the goal
        result = self._driver.send_rcu_command(goal.enable_charge)

        goal_failed = False
        if not result:
            goal_failed = True
            fail_msg = "Failed: Failed send RCU command."

        # Wait until command is received by Xnergy RCU Unit
        rospy.sleep(rospy.Duration(0.1))
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

            # when RCU unit status is in 'stop', 'error', 'debug_mode', the action will be aborted
            if (rcu_state == ChargerState.RCU_STOP and goal.enable_charge) or \
                (rcu_state == ChargerState.RCU_RESERVE_2) or \
                (rcu_state == ChargerState.RCU_ERROR) or \
                (rcu_state == ChargerState.RCU_RESERVE_1 and goal.enable_charge) or \
                    (rcu_state == ChargerState.RCU_IDLE and goal.enable_charge and timer > 4 and not self._driver.comm_interface == "gpio"):
                fail_msg = "RCU state is " + feedback_msg.status.message
                rospy.loginfo(fail_msg)
                goal_failed = True
                break
            # when RCU unit is successfully trigger charging or discharge, the action will be set as success
            elif (goal.enable_charge == True and rcu_state == ChargerState.RCU_CHARGING and self._driver.comm_interface == "canbus" and timer > 20) or \
                (goal.enable_charge == True and rcu_state == ChargerState.RCU_CHARGING) or \
                    (goal.enable_charge == False and rcu_state == ChargerState.RCU_IDLE):
                goal_failed = False
                break
            # check that preempt has not been requested by the client
            elif self._as.is_preempt_requested():
                fail_msg = "Failed: Goal was cancelled."
                rospy.loginfo(fail_msg)
                break
            elif timer >= 30:
                fail_msg = "Failed: Action timeout, it took more than 15 seconds."
                rospy.loginfo(fail_msg)
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
            fail_msg = fail_msg 
            if(len(errors) > 0):
                fail_msg = fail_msg + ":" + ";".join(errors)
            result_msg.success = False
            result_msg.message = fail_msg
            if(goal.enable_charge):
                rospy.logwarn("ChargeActionServer: Failed to enable charging.")
            else:
                rospy.logwarn(
                    "ChargeActionServer: Failed to disable charging.")
            self._as.set_aborted(result_msg)
