#!/usr/bin/env python3

import can
import rospy
from threading import Lock
from xnergy_charger_rcu.msg import ChargerState

# CAN BUS Command
_CANBUS_ADDRESS_ENABLE_CHARGING = [0x2F, 0x00, 0x20, 0x01, 0x01, 0x00, 0x00, 0x00]
_CANBUS_ADDRESS_DISABLE_CHARGING = [0x2F, 0x00, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00]

_CANBUS_ADDRESS_READ_CHARGING_STATUS = [0x40, 0x00, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00]

_CANBUS_ADDRESS_READ_SHADOW_ERROR_LO = [0x40, 0x02, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00]
_CANBUS_ADDRESS_READ_SHADOW_ERROR_HI = [0x40, 0x02, 0x20, 0x02, 0x00, 0x00, 0x00, 0x00]
_CANBUS_ADDRESS_READ_RUNTIME_ERROR_LO = [0x40, 0x02, 0x20, 0x03, 0x00, 0x00, 0x00, 0x00]
_CANBUS_ADDRESS_READ_RUNTIME_ERROR_HI = [0x40, 0x02, 0x20, 0x04, 0x00, 0x00, 0x00, 0x00]

_CANBUS_ADDRESS_READ_BATTERY_VOLTAGE = [0x40, 0x02, 0x20, 0x06, 0x00, 0x00, 0x00, 0x00]
_CANBUS_ADDRESS_READ_CHARGING_CURRENT = [0x40, 0x02, 0x20, 0x07, 0x00, 0x00, 0x00, 0x00]

_CANBUS_ADDRESS_READ_RUNTIME_VOLTAGE_SETTING = [0x40, 0x01, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00]
_CANBUS_ADDRESS_READ_RUNTIME_CURRENT_SETTING = [0x40, 0x01, 0x20, 0x02, 0x00, 0x00, 0x00, 0x00]


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
	def __init__(self, port='can0', bitrate=250000):
		""" Initialize RCU CANbus rtu """
		self.is_connected = True
		self.firmware_version_number = -1
		self.battery_voltage = -1
		self.output_current = -1
		self.runtime_voltage_setting = -1
		self.runtime_current_setting = -1
		self.charge_status = -1
		self.charge_status_message = ''
		self.errors = []
		self.device_temperature = -1
		self.coil_temperature = -1
		self.error_code = 0
		self.shadow_error_code = 0
		self.rcu_lock = Lock()
		self._port = port
		self._bitrate = bitrate

	def connect(self):
		try:
			self.bus = can.Bus(interface='socketcan', channel=self._port, bitrate=self._bitrate, receive_own_message=True)
			self.bus.send(can.Message(arbitration_id=0x60a, is_extended_id=False, data=_CANBUS_ADDRESS_READ_CHARGING_STATUS), timeout=0.2)
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
			self.bus.send(can.Message(arbitration_id=0x60a, is_extended_id=False, data=address), timeout=0.2)
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
			self.bus.send(can.Message(arbitration_id=0x60a, is_extended_id=False, data=_CANBUS_ADDRESS_READ_CHARGING_STATUS), timeout=0.2)
			reply_msg = self._receive_can_message()
			if reply_msg == '4f00200101000000':
				self.charge_status_message = 'charging'
				self.charge_status = ChargerState.RCU_CHARGING
			elif reply_msg == '4f00200100000000':
				self.charge_status_message = 'idle'
				self.charge_status = ChargerState.RCU_IDLE

			# Get Battery Voltage
			self.bus.send(can.Message(arbitration_id=0x60a, is_extended_id=False, data=_CANBUS_ADDRESS_READ_BATTERY_VOLTAGE), timeout=0.2)
			msg = self._receive_can_message()
			self.battery_voltage = int(f'{msg[10:12]}{msg[8:10]}', 16) / 128

			# Get Charging Current
			self.bus.send(can.Message(arbitration_id=0x60a, is_extended_id=False, data=_CANBUS_ADDRESS_READ_CHARGING_CURRENT), timeout=0.2)
			msg = self._receive_can_message()
			self.output_current = int(f'{msg[10:12]}{msg[8:10]}', 16) / 128

			# Get runtime error code
			self.bus.send(can.Message(arbitration_id=0x60a, is_extended_id=False, data=_CANBUS_ADDRESS_READ_RUNTIME_ERROR_HI), timeout=0.2)
			msg = self._receive_can_message()
			error_code_hi = int(f'{msg[10:12]}{msg[8:10]}', 16)

			self.bus.send(can.Message(arbitration_id=0x60a, is_extended_id=False, data=_CANBUS_ADDRESS_READ_RUNTIME_ERROR_LO), timeout=0.2)
			msg = self._receive_can_message()
			error_code_lo = int(f'{msg[10:12]}{msg[8:10]}', 16)

			self.error_code = error_code_hi << 16 | error_code_lo

			# todo decode error code

			# Get shadow error code
			self.bus.send(can.Message(arbitration_id=0x60a, is_extended_id=False, data=_CANBUS_ADDRESS_READ_SHADOW_ERROR_HI), timeout=0.2)
			msg = self._receive_can_message()
			shadow_error_code_hi = int(f'{msg[10:12]}{msg[8:10]}', 16)

			self.bus.send(can.Message(arbitration_id=0x60a, is_extended_id=False, data=_CANBUS_ADDRESS_READ_SHADOW_ERROR_LO), timeout=0.2)
			msg = self._receive_can_message()
			shadow_error_code_lo = int(f'{msg[10:12]}{msg[8:10]}', 16)

			self.shadow_error_code = shadow_error_code_hi << 16 | shadow_error_code_lo

			# todo decode shadow error code
			# todo get RCU temperature

			# get runtime voltage setting
			self.bus.send(can.Message(arbitration_id=0x60a, is_extended_id=False, data=_CANBUS_ADDRESS_READ_RUNTIME_VOLTAGE_SETTING), timeout=0.2)
			msg = self._receive_can_message()
			self.runtime_voltage_setting = int(f'{msg[10:12]}{msg[8:10]}', 16) / 128

			# get runtime current setting
			self.bus.send(can.Message(arbitration_id=0x60a, is_extended_id=False, data=_CANBUS_ADDRESS_READ_RUNTIME_CURRENT_SETTING), timeout=0.2)
			msg = self._receive_can_message()
			self.runtime_current_setting = int(f'{msg[10:12]}{msg[8:10]}', 16) / 128

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
