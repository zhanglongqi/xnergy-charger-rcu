#!/usr/bin/env python3

import can
import rospy
from threading import Lock
from xnergy_charger_rcu.msg import ChargerState

# CAN BUS Command
_CANBUS_ADDRESS_ENABLE_CHARGING = [
    0X2F, 0X00, 0X20, 0X01, 0X01, 0X00, 0X00, 0X00]
_CANBUS_ADDRESS_DISABLE_CHARGING = [
    0X2F, 0X00, 0X20, 0X01, 0X00, 0X00, 0X00, 0X00]
_CANBUS_ADDRESS_READ_CHARGING_STATUS = [
    0X40, 0X00, 0X20, 0X01, 0X00, 0X00, 0X00, 0X00]
_CANBUS_ADDRESS_READ_BATTERY_VOLTAGE = [
    0X40, 0X02, 0X20, 0X06, 0X00, 0X00, 0X00, 0X00]
_CANBUS_ADDRESS_READ_CHARGING_CURRENT = [
    0X40, 0X02, 0X20, 0X07, 0X00, 0X00, 0X00, 0X00]


class RCUCANbusAdapter:
    """ 
    RCU CANBUS Adapter
    This is the python class object for Xnergy RCU ROS Driver CANBUS adapter class.
    RCUCanbus Adapter python class object require to input serialport to initiate the class object
    RCUCanbusAdapter class object generally provide three function:
    a) Trigger RCU unit to charge through CANBUS
    b) Trigger RCU unit to discharge through CANBUS
    c) Trigger RCU unit to update status and capture the information into RCUCANbusAdapter object variables 
    """

    def __init__(self, port='can0'):
        """ Initialize RCU CANbus rtu """
        self.is_connected = True
        self.firmware_version_number = -1
        self.battery_voltage = -1
        self.runtime_voltage_setting = -1
        self.runtime_current_setting = -1
        self.charge_status = -1
        self.charge_status_message = 'None'
        self.errors = []
        self.device_temperature = -1
        self.coil_temperature = -1
        self.rcu_lock = Lock()
        self._port = port

    def connect(self):
        try:
            self.bus = can.Bus(interface='socketcan',
                               channel=self._port, receive_own_message=True)
            self.bus.send(can.Message(arbitration_id=0x60a, is_extended_id=False,
                                      data=_CANBUS_ADDRESS_READ_CHARGING_STATUS), timeout=0.2)
            test = self.bus.recv(timeout=1)
            if test is None:
                self.is_connected = False
                return False
            return True
        except:
            self.is_connected = False
            return False

    def enable_charge(self):
        """ Enable RCU to charge """
        return self.canbus_send(_CANBUS_ADDRESS_ENABLE_CHARGING)

    def disable_charge(self):
        """ disable RCU to charge """
        return self.canbus_send(_CANBUS_ADDRESS_DISABLE_CHARGING)

    def canbus_send(self, address):
        """ Sending command to RCU unit with threading lock to prevent parallel access to interface """
        self.rcu_lock.acquire()
        result = False
        try:
            self.bus.send(can.Message(arbitration_id=0x60a, is_extended_id=False,
                                      data=address), timeout=0.2)
            result = True
        except Exception as e:
            rospy.logwarn("CAN communication error: %s.", str(e))
            self.connect()
        self.rcu_lock.release()
        return result

    def reset_errors(self):
        self.errors = []
        
    def get_rcu_status(self):
        """ 
        Get charging and hardware information from RCU unit
        and store the information to RCUCANbusAdapter object variable
        """
        self.rcu_lock.acquire()
        try:
            # Get charging status
            self.bus.send(can.Message(arbitration_id=0x60a, is_extended_id=False,
                                      data=_CANBUS_ADDRESS_READ_CHARGING_STATUS), timeout=0.2)
            reply_msg = self._receive_can_message()
            if reply_msg == '4f00200101000000':
                self.charge_status_message = 'charging'
                self.charge_status = ChargerState.RCU_CHARGING
            elif reply_msg == '4f00200100000000':
                self.charge_status_message = 'idle'
                self.charge_status = ChargerState.RCU_IDLE

            # Get Battery Voltage
            self.bus.send(can.Message(arbitration_id=0x60a, is_extended_id=False,
                                      data=_CANBUS_ADDRESS_READ_BATTERY_VOLTAGE), timeout=0.2)

            battery_msg = self._receive_can_message()
            battery_msg = battery_msg[10:12]+battery_msg[8:10]

            self.battery_voltage = int(battery_msg, 16)

            # Get Charging Current
            self.bus.send(can.Message(arbitration_id=0x60a, is_extended_id=False,
                                      data=_CANBUS_ADDRESS_READ_CHARGING_CURRENT), timeout=0.2)
            charge_current_msg = self._receive_can_message()
            charge_current_msg = charge_current_msg[10:12] + \
                charge_current_msg[8:10]
            self.output_current = int(
                charge_current_msg[:4], 16)
        except can.CanError as e:
            self.charge_status_message = "comm error"
            self.charge_status = ChargerState.RCU_NOT_CONNECTED
            rospy.logerr("Cannot communicate with the device: %s.", str(e))
            self.connect()
        except:
            self.charge_status_message = "comm error"
            self.charge_status = ChargerState.RCU_NOT_CONNECTED
            rospy.logerr("Did not receive reply.")
            self.connect()
        self.rcu_lock.release()

    def _receive_can_message(self):
        reply_msg = None
        reply_msg = self.bus.recv(timeout=1)
        if reply_msg is None:
            raise Exception("Did not receive reply.")
        else:
            return reply_msg.data.hex()
