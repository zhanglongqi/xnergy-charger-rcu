#!/usr/bin/env python3
import struct
import time
from threading import RLock
from typing import Optional, Tuple

import can
import rospy
from can.exceptions import CanOperationError

from xnergy_charger_rcu.msg import ChargerState
from xnergy_charger_rcu.utils import (absolut_zero_temperature,
                                      translate_charge_status)

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
_CANBUS_ADDRESS_READ_MAIN_STATE = [0x40, 0x02, 0x20, 0x0A, 0x00, 0x00, 0x00, 0x00]
_CANBUS_ADDRESS_READ_FIRMWARE_REVISION_LO = [0x40, 0x02, 0x20, 0x0B, 0x00, 0x00, 0x00, 0x00]
_CANBUS_ADDRESS_READ_FIRMWARE_REVISION_HI = [0x40, 0x02, 0x20, 0x0C, 0x00, 0x00, 0x00, 0x00]

_CANBUS_ADDRESS_READ_RUNTIME_VOLTAGE_SETTING = [0x40, 0x01, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00]
_CANBUS_ADDRESS_READ_RUNTIME_CURRENT_SETTING = [0x40, 0x01, 0x20, 0x02, 0x00, 0x00, 0x00, 0x00]

_CANBUS_ADDRESS_REQUEST_RANGE_CHECK = [0x2F, 0x00, 0x20, 0x02, 0x01, 0x00, 0x00, 0x00]
_CANBUS_ADDRESS_READ_RANGE_CHECK_STATUS = [0x40, 0x02, 0x20, 0x0D, 0x00, 0x00, 0x00, 0x00]


class CanDataError(Exception):
	"""
	if the received data is not valid, this exception will be raised

    If specified, the error code is automatically appended to the message:

    >>> # With an error code (it also works with a specific error):
    >>> error = CanOperationError(message="Failed to do the thing", error_code=42)
    >>> str(error)
    'Failed to do the thing [Error Code 42]'
    >>>
    >>> # Missing the error code:
    >>> plain_error = CanError(message="Something went wrong ...")
    >>> str(plain_error)
    'Something went wrong ...'

    :param error_code:
        An optional error code to narrow down the cause of the fault

    :arg error_code:
        An optional error code to narrow down the cause of the fault
    """

	def __init__(
		self,
		message: str = "",
		error_code: Optional[int] = None,
	) -> None:
		self.error_code = error_code
		super().__init__(message if error_code is None else f"{message} [Error Code {error_code}]")


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

	def __init__(self, port='can0', node_id=0xA):
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
		self.device_temperature = absolut_zero_temperature
		self.coil_temperature = absolut_zero_temperature
		self.error_code = 0
		self.shadow_error_code = 0
		self.rcu_lock = RLock()
		self._port = port
		self._NODE_ID = node_id
		self._CAN_FILTERS = [{"can_id": 0x580 + node_id, "can_mask": 0x1FFFFFFF, "extended": False}]
		self.lost_packet_count = 0
		self.lost_packet_tolerence = 2

	def connect(self):
		try:
			self.bus = can.ThreadSafeBus(interface='socketcan', channel=self._port, receive_own_message=True, can_filters=self._CAN_FILTERS)
			valid, data = self.canopen_sdo(_CANBUS_ADDRESS_READ_FIRMWARE_REVISION_HI)
			if valid and data:
				firmware_version_hi = struct.unpack('=HH', data)[0]
				self.firmware_version_number = firmware_version_hi
				rospy.loginfo(f'firmware_version_hi: {firmware_version_hi:04X}')
				self.is_connected = True
				return True
			else:
				self.is_connected = False
				return False
		except (can.exceptions.CanInterfaceNotImplementedError, can.exceptions.CanInitializationError, ValueError) as e:
			rospy.logerr(f'RCU CANbus connection failed: {e}')
			self.is_connected = False
			return False

	def enable_charge(self):
		""" Enable RCU to charge """
		valid, data = self.canopen_sdo(_CANBUS_ADDRESS_ENABLE_CHARGING)
		return valid

	def disable_charge(self):
		""" disable RCU to charge """
		valid, data = self.canopen_sdo(_CANBUS_ADDRESS_DISABLE_CHARGING)
		return valid

	def request_range_check(self) -> bool:
		"""
		send request to RCU for range checking
		return:
		True if request get ACK
		False if no ACK
		"""
		valid, data = self.canopen_sdo(_CANBUS_ADDRESS_REQUEST_RANGE_CHECK)

		return valid

	def read_range_check_status(self) -> Tuple[bool, int]:
		"""
		read the range check status of the RCU unit
		return status after ACK
		return None if no ACK
		"""
		valid, data = self.canopen_sdo(_CANBUS_ADDRESS_READ_RANGE_CHECK_STATUS)

		check_status = struct.unpack('=HH', data)[0] if valid else None

		return valid, check_status

	def reset_errors(self):
		self.errors = []

	def parse_msg(self, msg, req):
		""" Parse RCU CANbus message
		return: (index_valid, data)
		 """
		try:
			(command, index, sub_index, data) = struct.unpack('=BHB4s', msg)
		except struct.error:
			return False, None
		index_expected = req[2] << 8 | req[1]
		sub_index_expected = req[3]
		# rospy.loginfo(f'{binascii.hexlify(msg)}')
		# rospy.loginfo(f'index:{msg[1:3]} ->{index}, sub_index: {msg[3:4]} -> {sub_index}, index_expected: {index_expected}, sub_index_expected: {sub_index_expected}')

		if index == index_expected and sub_index == sub_index_expected:
			return (True, data)
		else:
			return (False, data)

	def canopen_sdo(self, cmd):
		""" Sending command to RCU unit with threading lock to prevent parallel access to interface
			check the index and sub index of the message
			return the index check result and the data part
		"""
		self.rcu_lock.acquire()
		return_valid = False
		return_result = bytearray()
		try:
			msg_tx = can.Message(timestamp=time.time(), arbitration_id=0x600 + self._NODE_ID, is_extended_id=False, data=cmd)
			self.bus.send(msg=msg_tx, timeout=0.2)

			got = False
			timer = time.time()
			while not got:
				if time.time() - timer > 0.5:
					self.lost_packet_count += 1
					raise CanDataError('RCU CANBUS RX timeout')

				msg_rx = self.bus.recv(timeout=0.1)
				if msg_rx is not None and not msg_rx.is_error_frame:
					break
				else:
					continue

			msg = msg_rx.data
			valid, data = self.parse_msg(msg, cmd)
			if not valid:
				rospy.logdebug(f'CAN message \n{msg_tx=}, \n{msg_rx=}')
				self.lost_packet_count += 1
			else:
				self.lost_packet_count = 0
			return_valid = valid
			return_result = data

		except (CanOperationError, CanDataError) as e:
			rospy.logwarn(f'CAN communication error: {e}')
			return_valid = False
			return_result = None
			self.lost_packet_count += 1
		finally:
			self.rcu_lock.release()
			if self.lost_packet_count > self.lost_packet_tolerence:
				self.charge_status = ChargerState.RCU_NOT_CONNECTED
				self.charge_status_message = "comm error"
				self.is_connected = False
				rospy.logerr(f'lost {self.lost_packet_count} packets continusly, please check your connection and power up RCU unit')
				self.bus.flush_tx_buffer()
				self.bus.recv(timeout=0.1)
				return False, None

		return return_valid, return_result

	def _get_rcu_status_single(self, cmd) -> Tuple[bool, bytearray | None]:
		"""
		get the status of the RCU unit
		return status after ACK
		return None if no ACK
		"""
		if not self.is_connected:
			return (False, None)

		valid, data = self.canopen_sdo(cmd)
		if valid and data:
			return (True, data)
		else:
			return (False, data)

	def get_rcu_status(self):
		"""
        Get charging and hardware information from RCU unit
        and store the information to RCUCANbusAdapter object variable
        """

		# Get charging status
		valid, data = self._get_rcu_status_single(_CANBUS_ADDRESS_READ_MAIN_STATE)
		if valid and data:
			main_state = struct.unpack('=HH', data)[0]
			self.charge_status = main_state
			self.charge_status_message = translate_charge_status(main_state)
			rospy.logdebug(f'charger state: {self.charge_status} {self.charge_status_message}')
		else:
			rospy.logwarn('Failed to get main state')
			return

		# Get Battery Voltage
		valid, data = self._get_rcu_status_single(_CANBUS_ADDRESS_READ_BATTERY_VOLTAGE)
		if valid and data:
			self.battery_voltage = struct.unpack('=HH', data)[0] / 128
			rospy.logdebug(f'battery_voltage: {self.battery_voltage:6.2f}')
		else:
			rospy.logwarn('Failed to get Battery Voltage')
			return

		# Get Charging Current
		valid, data = self._get_rcu_status_single(_CANBUS_ADDRESS_READ_CHARGING_CURRENT)
		if valid and data:
			self.output_current = struct.unpack('=HH', data)[0] / 128
			rospy.logdebug(f'output_current: {self.output_current:6.2f}')
		else:
			rospy.logwarn('Failed to get Battery Voltage')
			return

		# Get runtime error code
		valid, data = self._get_rcu_status_single(_CANBUS_ADDRESS_READ_RUNTIME_ERROR_HI)
		if valid and data:
			error_code_hi = struct.unpack('=HH', data)[0]
			# rospy.logdebug(f'error_code_hi: {error_code_hi:04X}')
		else:
			error_code_hi = 0
			rospy.logwarn('Failed to get error_code_hi')
			return

		valid, data = self._get_rcu_status_single(_CANBUS_ADDRESS_READ_RUNTIME_ERROR_LO)
		if valid and data:
			error_code_lo = struct.unpack('=HH', data)[0]
			# rospy.logdebug(f'error_code_lo: {error_code_lo:04X}')
		else:
			error_code_lo = 0
			rospy.logwarn('Failed to get error_code_lo')
			return

		self.error_code = error_code_hi << 16 | error_code_lo
		rospy.logdebug(f'error_code: {self.error_code :08X}')

		# todo decode error code

		# Get shadow error code
		valid, data = self._get_rcu_status_single(_CANBUS_ADDRESS_READ_SHADOW_ERROR_HI)
		if valid and data:
			shadow_error_code_hi = struct.unpack('=HH', data)[0]
			# rospy.logdebug(f'shadow_error_code_hi: {shadow_error_code_hi:04X}')
		else:
			shadow_error_code_hi = 0
			rospy.logwarn('Failed to get shadow_error_code_hi')
			return

		valid, data = self._get_rcu_status_single(_CANBUS_ADDRESS_READ_SHADOW_ERROR_LO)
		if valid and data:
			shadow_error_code_lo = struct.unpack('=HH', data)[0]
			# rospy.logdebug(f'shadow_error_code_lo: {shadow_error_code_lo:04X}')
		else:
			shadow_error_code_lo = 0
			rospy.logwarn('Failed to get shadow_error_code_lo')
			return

		self.shadow_error_code = shadow_error_code_hi << 16 | shadow_error_code_lo
		rospy.logdebug(f'shadow_error_code: {self.shadow_error_code:08X}')

		# todo decode shadow error code
		# todo get RCU temperature

		# get runtime voltage setting
		valid, data = self._get_rcu_status_single(_CANBUS_ADDRESS_READ_RUNTIME_VOLTAGE_SETTING)
		if valid and data:
			self.runtime_voltage_setting = struct.unpack('=HH', data)[0] / 128
			rospy.logdebug(f'runtime_voltage_setting: {self.runtime_voltage_setting:6.2f}')
		else:
			rospy.logwarn('Failed to get runtime_voltage_setting')
			return

		# get runtime current setting
		valid, data = self._get_rcu_status_single(_CANBUS_ADDRESS_READ_RUNTIME_CURRENT_SETTING)
		if valid and data:
			self.runtime_current_setting = struct.unpack('=HH', data)[0] / 128
			rospy.logdebug(f'runtime_current_setting: {self.runtime_current_setting:6.2f}')
		else:
			rospy.logwarn('Failed to get runtime_current_setting')
			return
