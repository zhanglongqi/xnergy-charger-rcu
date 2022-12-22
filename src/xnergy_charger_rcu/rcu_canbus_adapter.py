#!/usr/bin/env python3
import struct
import time
from dataclasses import dataclass
from queue import Queue
from threading import Thread
from typing import Optional, Tuple, Union

import can
import rospy
from can.exceptions import CanOperationError

from xnergy_charger_rcu.msg import ChargerState
from xnergy_charger_rcu.utils import (absolut_zero_temperature,
                                      translate_charge_status)

# CAN BUS Command
_CANBUS_ADDRESS_ENABLE_CHARGING = [0x2F, 0x00, 0x20, 0x01, 0x01, 0x00, 0x00, 0x00]
_CANBUS_ADDRESS_DISABLE_CHARGING = [0x2F, 0x00, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00]

_CANBUS_ADDRESS_REQUEST_RANGE_CHECK = [0x2F, 0x00, 0x20, 0x02, 0x01, 0x00, 0x00, 0x00]


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


@dataclass
class CANOPEN_SDO_OBJ:
	name: str
	index: int
	sub_index: int
	value: bytes


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
		self.errors = []
		self.device_temperature = absolut_zero_temperature
		self.coil_temperature = absolut_zero_temperature
		self._port = port
		self._NODE_ID = node_id
		self._CAN_FILTERS = [
			{  # SDO response
				"can_id": 0x580 + node_id,
				"can_mask": 0x7FF,
				"extended": False
			},
			{  # heartbeat 
				"can_id": 0x700 + node_id,
				"can_mask": 0x7FF,
				"extended": False
			},
		]
		self.lost_packet_count = 0
		self.lost_packet_tolerence = 20

		self.sdo_od = {  # object dictionary
			'main_state': CANOPEN_SDO_OBJ('main_state', 0x2002, 0x0A, b'\x00\x00\x00\x00'),
			#
			'output_voltage': CANOPEN_SDO_OBJ('output_voltage', 0x2002, 0x05, b'\x00\x00\x00\x00'),
			'battery_voltage': CANOPEN_SDO_OBJ('battery_voltage', 0x2002, 0x06, b'\x00\x00\x00\x00'),
			'output_current': CANOPEN_SDO_OBJ('output_current', 0x2002, 0x07, b'\x00\x00\x00\x00'),
			#
			'shadow_error_code_lo': CANOPEN_SDO_OBJ('shadow_error_code_lo', 0x2002, 0x01, b'\x00\x00\x00\x00'),
			'shadow_error_code_hi': CANOPEN_SDO_OBJ('shadow_error_code_hi', 0x2002, 0x02, b'\x00\x00\x00\x00'),
			'runtime_error_code_lo': CANOPEN_SDO_OBJ('runtime_error_code_lo', 0x2002, 0x03, b'\x00\x00\x00\x00'),
			'runtime_error_code_hi': CANOPEN_SDO_OBJ('runtime_error_code_hi', 0x2002, 0x04, b'\x00\x00\x00\x00'),
			#
			'firmware_version_lo': CANOPEN_SDO_OBJ('firmware_version_lo', 0x2002, 0x0B, b'\x00\x00\x00\x00'),
			'firmware_version_hi':  CANOPEN_SDO_OBJ('firmware_version_hi', 0x2002, 0x0C, b'\x00\x00\x00\x00'),
			#
			'range_check_status': CANOPEN_SDO_OBJ('charger_range_status', 0x2002, 0x0D, b'\x00\x00\x00\x00'),
			#
			'runtime_voltage_setting': CANOPEN_SDO_OBJ('runtime_voltage_setting', 0x2001, 0x01, b'\x00\x00\x00\x00'),
			'runtime_current_setting': CANOPEN_SDO_OBJ('runtime_current_setting', 0x2001, 0x02, b'\x00\x00\x00\x00'),
		}

		self.bus = None
		self._queue_write = Queue()
		self._can_t = Thread(target=self.canbus_worker, daemon=True)
		self._can_t.start()

	def connect(self):
		try:
			self.bus = can.ThreadSafeBus(interface='socketcan', channel=self._port, receive_own_message=False, can_filters=self._CAN_FILTERS)
			self.is_connected = True
			self.lost_packet_count = 0
			return True
		except (can.exceptions.CanInterfaceNotImplementedError, can.exceptions.CanInitializationError, ValueError) as e:
			rospy.logerr(f'RCU CANbus connection failed: {e}')
			self.is_connected = False
			return False

	def canbus_send_rcv(self, msg_tx) -> Union[can.Message, None]:
		msg_rx = None
		try:
			self.bus.send(msg=msg_tx, timeout=0.01)
			msg_rx = self.bus.recv(timeout=0.03)
			if msg_rx is None or msg_rx.is_error_frame:
				rospy.logwarn(f'CAN comm error: {msg_rx=}')
				self.lost_packet_count += 1
		except CanOperationError as e:
			rospy.logwarn(f'CAN comm error: {e}')
		return msg_rx

	def canbus_worker(self):
		rospy.logdebug('canbus worker started')

		while not self.is_connected or self.bus is None:
			rospy.logdebug('CAN bus is not esstablished yet.')
			rospy.sleep(1)

		have_heartbeat = False  # keep it compitiable for old firmware version that have no heartbeat
		detect_window_timer = time.time()
		detect_window_timeout = 2
		rospy.loginfo('detecting heartbeat from RCU')

		while time.time() - detect_window_timer <= detect_window_timeout:
			try:
				res = self.bus.recv(timeout=0.01)  # heartbeat interval is 100 ms
			except CanOperationError as e:
				rospy.logwarn(e)
				continue
			if res and self.is_heartbeat(res):
				have_heartbeat = True
				break
		rospy.loginfo(f'{ "Detected" if have_heartbeat else "NO" } heartbeat from RCU')

		SNEDING_GROUP_MAX = 3

		while True:
			window_is_open = False
			if have_heartbeat:
				try:
					res = self.bus.recv(timeout=0.01)
				except CanOperationError:
					window_is_open = False
				else:
					if res and self.is_heartbeat(res):
						# just got heartbeat, got window for query routine
						window_is_open = True
					else:
						window_is_open = False
			else:
				rospy.sleep(0.1)
				window_is_open = True

			if window_is_open and self.is_connected:
				sending_counter = 0
				while not self._queue_write.empty() and sending_counter <= SNEDING_GROUP_MAX:
					sending_counter += 1
					msg_tx = self._queue_write.get()
					msg_rx = self.canbus_send_rcv(msg_tx)
					if msg_rx and not msg_rx.is_error_frame:
						response_valid, index_rx, sub_index_rx, data_rx = self.parse_sdo(msg_rx, msg_tx)
						if response_valid:
							self.lost_packet_count = 0
							self.update_registers(index_rx, sub_index_rx, data_rx)
							self._queue_write.task_done()
						else:
							self.lost_packet_count += 1
							rospy.logdebug('invalid response from RCU')
					else:
						self._queue_write.put(msg_tx)
						self.lost_packet_count += 1
						rospy.logwarn(f'comm failed. will retry soon. {msg_rx=} {msg_tx=}')
			if self.lost_packet_count > self.lost_packet_tolerence:
				self.is_connected = False
				rospy.logerr(f'lost {self.lost_packet_count} packets continusly, please check your connection and power up RCU unit')
				self.bus.flush_tx_buffer()
				rospy.sleep(2)

	def enable_charge(self):
		""" Enable RCU to charge """
		self.canopen_sdo(_CANBUS_ADDRESS_ENABLE_CHARGING)
		return True

	def disable_charge(self):
		""" disable RCU to charge """
		self.canopen_sdo(_CANBUS_ADDRESS_DISABLE_CHARGING)
		return True

	def request_range_check(self) -> bool:
		"""
		send request to RCU for range checking
		"""
		self.canopen_sdo(_CANBUS_ADDRESS_REQUEST_RANGE_CHECK)
		return True

	def read_range_check_status(self) -> Tuple[bool, int]:
		"""
		read the range check status of the RCU unit
		"""
		if self.is_connected:
			return True, self.range_check_status
		else:
			return False, self.range_check_status

	def reset_errors(self):
		self.errors = []

	def update_registers(self, index, sub_index, value_raw):
		# rospy.logdebug(f'{index=}, {sub_index=}, {value_raw=}')
		for name, obj in self.sdo_od.items():
			if index == obj.index and sub_index == obj.sub_index:
				obj.value = value_raw

	def is_heartbeat(self, msg_rx: can.Message) -> bool:
		""" Parse RCU CANbus message
		"""
		# rospy.logdebug(f'{msg_rx=}')
		func_code_rx = (msg_rx.arbitration_id >> 7) & 0xF
		node_id_rx = msg_rx.arbitration_id & 0x7F
		data_rx = msg_rx.data
		# rospy.logdebug(f'{func_code_rx=:0>4b} {node_id_rx=:02X} {data_rx.hex()=:}')
		if 0b1110 == func_code_rx and node_id_rx == self._NODE_ID and bytearray(b'\x05') == data_rx:  #  heartbeat response
			return True
		else:
			return False

	def parse_sdo(self, msg_rx: can.Message, msg_tx: can.Message) -> Tuple[bool, int, int, bytes]:
		""" Parse RCU CANbus message
		return: (index_valid, data)
		"""
		# rospy.logdebug(f'{msg_rx=}')
		func_code_rx = (msg_rx.arbitration_id >> 7) & 0xF
		if 0b1011 != func_code_rx:  #  not SDO response
			rospy.logdebug(f'function code is not 0b1011, not SDO {msg_rx=}')
			return False, 0, 0, b''

		data_rx = msg_rx.data
		node_id_rx = msg_rx.arbitration_id & 0x7F
		data_tx = msg_tx.data
		node_id_tx = msg_tx.arbitration_id & 0x7F

		try:
			(command_rx, index_rx, sub_index_rx, od_data_rx) = struct.unpack('=BHB4s', data_rx)
			(command_tx, index_tx, sub_index_tx, od_data_tx) = struct.unpack('=BHB4s', data_tx)
		except struct.error:
			rospy.logdebug(f'data is invalid, can not unpack. {data_rx.hex()=}, {data_tx.hex()=}')
			return False, 0, 0, b''

		# rospy.loginfo(f'{binascii.hexlify(msg)}')
		# rospy.loginfo(f'index:{msg[1:3]} ->{index}, sub_index: {msg[3:4]} -> {sub_index}, index_expected: {index_expected}, sub_index_expected: {sub_index_expected}')

		# if index_rx == index_tx and sub_index_rx == sub_index_tx and node_id_rx == node_id_tx:
		return True, index_rx, sub_index_rx, od_data_rx
		# else:
		# 	rospy.logwarn(f'the msg rx is not from the msg tx. {msg_rx=}, {msg_tx=}')
		# 	return False, 0, 0, b''

	def canopen_sdo(self, cmd):
		""" Sending command to RCU unit
			check the index and sub index of the message
			return the index check result and the data part
		"""

		msg_tx = can.Message(timestamp=time.time(), arbitration_id=0x600 + self._NODE_ID, is_extended_id=False, data=cmd)
		self._queue_write.put(msg_tx)

	@property
	def charge_status(self):
		if self.is_connected:
			main_state = struct.unpack('=HH', self.sdo_od['main_state'].value)[0]
		else:
			main_state = ChargerState.RCU_NOT_CONNECTED
		return main_state

	@property
	def charge_status_message(self):
		if self.is_connected:
			main_state = struct.unpack('=HH', self.sdo_od['main_state'].value)[0]
			charge_status_message = translate_charge_status(main_state)
		else:
			main_state = ChargerState.RCU_NOT_CONNECTED
			charge_status_message = 'comm error'
		return charge_status_message

	@property
	def output_voltage(self):
		if self.is_connected:
			output_voltage = struct.unpack('=HH', self.sdo_od['output_voltage'].value)[0] / 128
			# rospy.logdebug(f'output_voltage: {output_voltage:6.2f}')
		else:
			output_voltage = -1
		return output_voltage

	@property
	def battery_voltage(self):
		if self.is_connected:
			battery_voltage = struct.unpack('=HH', self.sdo_od['battery_voltage'].value)[0] / 128
			# rospy.logdebug(f'battery_voltage: {battery_voltage:6.2f}')
		else:
			battery_voltage = -1
		return battery_voltage

	@property
	def runtime_voltage_setting(self):
		if self.is_connected:
			runtime_voltage_setting = struct.unpack('=HH', self.sdo_od['runtime_voltage_setting'].value)[0] / 128
			# rospy.logdebug(f'runtime_voltage_setting: {runtime_voltage_setting:6.2f}')
		else:
			runtime_voltage_setting = -1
		return runtime_voltage_setting

	@property
	def runtime_current_setting(self):
		if self.is_connected:
			runtime_current_setting = struct.unpack('=HH', self.sdo_od['runtime_current_setting'].value)[0] / 128
			# rospy.logdebug(f'runtime_current_setting: {runtime_current_setting:6.2f}')
		else:
			runtime_current_setting = -1
		return runtime_current_setting

	@property
	def output_current(self):
		if self.is_connected:
			output_current = struct.unpack('=HH', self.sdo_od['output_current'].value)[0] / 128
			# rospy.logdebug(f'output_current: {output_current:6.2f}')
		else:
			output_current = -1
		return output_current

	@property
	def shadow_error_code(self):
		if self.is_connected:
			shadow_error_code_lo = struct.unpack('=HH', self.sdo_od['shadow_error_code_lo'].value)[0]
			shadow_error_code_hi = struct.unpack('=HH', self.sdo_od['shadow_error_code_hi'].value)[0]
			shadow_error_code = shadow_error_code_hi << 16 | shadow_error_code_lo
			# rospy.logdebug(f'shadow_error_code: 0x{shadow_error_code:08X}')
		else:
			shadow_error_code = 0x00000000
		return shadow_error_code

	@property
	def error_code(self):
		if self.is_connected:
			error_code_lo = struct.unpack('=HH', self.sdo_od['runtime_error_code_lo'].value)[0]
			error_code_hi = struct.unpack('=HH', self.sdo_od['runtime_error_code_hi'].value)[0]
			error_code = error_code_hi << 16 | error_code_lo
			# rospy.logdebug(f'runtime_error_code: 0x{error_code:08X}')
		else:
			error_code = 0x00000000
		return error_code

	@property
	def firmware_version_number(self):
		if self.is_connected:
			firmware_version_lo = struct.unpack('=HH', self.sdo_od['firmware_version_lo'].value)[0]
			firmware_version_hi = struct.unpack('=HH', self.sdo_od['firmware_version_hi'].value)[0]
			firmware_version_number = firmware_version_hi << 16 | firmware_version_lo
			# rospy.logdebug(f'firmware_version_number: {firmware_version_number:08X}')
		else:
			firmware_version_number = 0x00000000
		return firmware_version_number

	@property
	def range_check_status(self):
		if self.is_connected:
			range_check_status = struct.unpack('=HH', self.sdo_od['range_check_status'].value)[0]
			rospy.logdebug(f'range_check_status: {range_check_status}')
		else:
			range_check_status = -1
		return range_check_status

	def get_rcu_status(self):
		"""
        Get charging and hardware information from RCU unit
        and store the information to RCUCANbusAdapter object variable
        """
		for name, obj in self.sdo_od.items():
			index_lo = obj.index & 0xFF
			index_hi = (obj.index >> 8) & 0xFF
			msg_tx = can.Message(timestamp=time.time(),
									arbitration_id=0x600 + self._NODE_ID,
									is_extended_id=False,
									data=[
										0x40,
										index_lo,
										index_hi,
										obj.sub_index,
										0x00,
										0x00,
										0x00,
										0x00,
									])
			self._queue_write.put(msg_tx)
