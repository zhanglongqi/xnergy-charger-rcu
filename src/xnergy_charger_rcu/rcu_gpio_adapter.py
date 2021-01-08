#!/usr/bin/env python3

import gpiod
import rospy
from threading import Lock
from xnergy_charger_rcu.msg import ChargerState


class RCUGPIOAdapter:
    """ 
    RCU GPIO Adapter
    This is the python class object for Xnergy RCU ROS Driver GPIO adapter class.
    RCU GPIO Adapter python class object require to input serialport to initiate the class object
    RCUGPIOAdapter class object generally provide three function:
    a) Trigger RCU unit to charge through GPIO
    b) Trigger RCU unit to discharge through GPIO
    c) Trigger RCU unit to update status and capture the information into RCUGPIOAdapter object variables 
    """

    def __init__(self, charger_control, charger_status):
        """ Initialize RCU GPIO """
        self.is_connected = True
        self.firmware_version_number = -1
        self.runtime_voltage_setting = -1
        self.runtime_current_setting = -1
        self.charge_status = -1
        self.battery_voltage = -1
        self.output_current = -1
        self.charge_status_message = 'None'
        self.errors = []
        self.device_temperature = -1
        self.coil_temperature = -1
        self._charger_control = charger_control
        self._charger_status = charger_status

    def connect(self):
        try:

            # retrieve charger control GPIO line
            config = gpiod.line_request()
            config.consumer = "ChargerControl"
            config.request_type = gpiod.line_request.DIRECTION_OUTPUT
            self.charger_en = gpiod.chip(self._charger_control['chip']).get_line(
                self._charger_control['line'])
            self.charger_en.request(config)

            # retrieve charger status GPIO line
            config = gpiod.line_request()
            config.consumer = "ChargerRead"
            config.request_type = gpiod.line_request.DIRECTION_INPUT
            self.charger_st = gpiod.chip(self._charger_status['chip']).get_line(
                self._charger_status['line'])
            self.charger_st.request(config)
            self.disable_charge()
            return True
        except:
            self.is_connected = False
            return False

    def enable_charge(self):
        """ Enable RCU to charge """
        self.charger_en.set_value(0)
        return True

    def disable_charge(self):
        """ disable RCU to charge """
        self.charger_en.set_value(1)
        return True

    def reset_errors(self):
        self.errors = []

    def get_rcu_status(self):
        """ 
        Get charging and hardware information from RCU unit
        and store the information to RCUGPIOAdapter object variable
        """
        charger_status = self.charger_st.get_value()
        if charger_status == 0:
            self.charge_status_message = 'charging'
            self.charge_status = ChargerState.RCU_CHARGING
        elif charger_status == 1:
            self.charge_status_message = 'idle'
            self.charge_status = ChargerState.RCU_IDLE
        else:
            self.charge_status_message = 'error'
            self.charge_status = ChargerState.RCU_NOT_CONNECTED
