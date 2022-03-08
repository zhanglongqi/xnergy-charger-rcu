#!/usr/bin/env python3

import rospy
import serial
import minimalmodbus
from minimalmodbus import Instrument, MODE_RTU
from xnergy_charger_rcu.msg import ChargerState
from threading import Lock
from xnergy_charger_rcu.utils import translate_charge_status, translate_error_code, absolut_zero_temperature

minimalmodbus.CLOSE_PORT_AFTER_EACH_CALL = True
# MODBUS function code
_MODBUS_READ_SINGLE_COIL = int("0x01", 16)
_MODBUS_READ_SINGLE_DISCRETE = int("0x02", 16)
_MODBUS_READ_HOLDING_REGISTERS = int("0x03", 16)
_MODBUS_READ_INPUT_REGISTERS = int("0x04", 16)
_MODBUS_WRITE_SINGLE_REGISTER = int("0x06", 16)

# Xnergy RCU Modbus Address
_MODBUS_INPUT_REGISTER_NUM = 150
_MODBUS_GROUP_SIZE = 50

# Modbus type: coil(read/write)
_MODBUS_ADDRESS_ENABLE_CHARGING = 0
_MODBUS_ADDRESS_REQUEST_RANGE_CHECK = 2

# Modbus type: input register (read only)
_MODBUS_ADDRESS_SHADOW_ERROR_LOW = 0
_MODBUS_ADDRESS_SHADOW_ERROR_HIGH = 1
_MODBUS_ADDRESS_DEVICE_TEMPERATURE = 36
_MODBUS_ADDRESS_COIL_TEMPERATURE = 37
_MODBUS_ADDRESS_OUTPUT_CURRENT = 31
_MODBUS_ADDRESS_BATTERY_VOLTAGE = 32
_MODBUS_ADDRESS_CHARGE_STATUS = 51
_MODBUS_ADDRESS_RCU_ERROR_LOW = 52
_MODBUS_ADDRESS_RCU_ERROR_HIGH = 53
_MODBUS_ADDRESS_FIRMWARE_VERSION_NUMBER_HI = 99
_MODBUS_ADDRESS_READ_RANGE_CHECK_STATUS = 100

# Modbus type: holding register (read/write)
_MODBUS_ADDRESS_RCU_DEVICE_ID = 1
_MODBUS_ADDRESS_RUNTIME_VOLTAGE_SETTING = 8
_MODBUS_ADDRESS_RUNTIME_CURRENT_SETTING = 9

rcu_error_data_list = [1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 16384, 32768]


class RCUModbusAdapter:
	"""
    RCU MODBUS interface
    This is the python class object for Xnergy RCU ROS Driver MODBUS RTU adapter class.
    RCUModbus Interface python class object require to input serialport to initiate the class object
    RCUModbusInterface class object generally provide three function:
    a) Trigger RCU unit to charge through MODBUS RTU
    b) Trigger RCU unit to discharge through MODBUS RTU
    c) Trigger RCU unit to update status and capture the information into RCUModbusInterface object variables
    """

	def __init__(self, port="/dev/ttyUSB0", slave_addr=16, baudrate=9600):
		""" Initialize RCU modbus rtu """
		self.is_connected = True
		self.firmware_version_number = -1
		self.battery_voltage = -1
		self.output_current = -1
		self.runtime_voltage_setting = -1
		self.runtime_current_setting = -1
		self.charge_status = -1
		self.charge_status_message = 'None'
		self.device_temperature = absolut_zero_temperature
		self.coil_temperature = absolut_zero_temperature
		self.rcu_lock = Lock()
		self._baudrate = baudrate
		self._slave_addr = slave_addr
		self._port = port
		self.error_code = 0
		self.shadow_error_code = 0
		self.errors = []
		self._previous_shadow_error_high = 0
		self._previous_shadow_error_low = 0
		self._regs_input = []

		self.lost_packet_count = 0
		self.lost_packet_tolerence = 2

	def connect(self):
		try:
			self._instrument = Instrument(port=self._port, mode=MODE_RTU, slaveaddress=self._slave_addr)
			self._instrument.serial.baudrate = self._baudrate
			self._instrument.serial.timeout = 1
			self._instrument.mode = MODE_RTU

			# RCU Hardware Setting

			# Read RCU Hardware Setting
			# firmware version information
			firmware_version_number_reg = self._instrument.read_registers(registeraddress=_MODBUS_ADDRESS_FIRMWARE_VERSION_NUMBER_HI,
																			number_of_registers=1,
																			functioncode=_MODBUS_READ_INPUT_REGISTERS)
			self.firmware_version_number = firmware_version_number_reg[0]

			# runtime voltage setting information
			runtime_voltage_setting_reg = self._instrument.read_registers(registeraddress=_MODBUS_ADDRESS_RUNTIME_VOLTAGE_SETTING,
																			number_of_registers=1,
																			functioncode=_MODBUS_READ_HOLDING_REGISTERS)
			self.runtime_voltage_setting = float(runtime_voltage_setting_reg[0] >> 7)

			# runtime currnet setting information
			runtime_current_setting_reg = self._instrument.read_registers(registeraddress=_MODBUS_ADDRESS_RUNTIME_CURRENT_SETTING,
																			number_of_registers=1,
																			functioncode=_MODBUS_READ_HOLDING_REGISTERS)
			self.runtime_current_setting = float(runtime_current_setting_reg[0] >> 7)
			self.is_connected = True
			self.lost_packet_count = 0
			return True
		except (serial.SerialException, minimalmodbus.ModbusException, AttributeError) as e:
			rospy.logerr(e)
			self.is_connected = False
			return False
		# self._instrument.serial.timeout = 0.4

	def enable_charge(self):
		""" Enable RCU to charge """
		return self.write_bit(_MODBUS_ADDRESS_ENABLE_CHARGING, 1)

	def disable_charge(self):
		""" Disable RCU to charge """
		return self.write_bit(_MODBUS_ADDRESS_ENABLE_CHARGING, 0)

	def request_range_check(self) -> bool:
		"""
		send request to RCU for range checking
		return:
		True if request is sent successfully
		False if request is failed
		"""
		valid = self.write_bit(_MODBUS_ADDRESS_REQUEST_RANGE_CHECK, 1)

		return valid

	def read_range_check_status(self) -> (bool, int):
		"""
		read the range check status of the RCU unit
		"""

		check_status = self._range_check_status

		return True, check_status

	def write_bit(self, register_address, input_value):
		""" Sending command to RCU unit with threading lock to prevent parallel access to interface """
		self.rcu_lock.acquire()
		result = False
		try:
			self._instrument.write_bit(registeraddress=register_address, value=input_value)
			result = True
		except serial.serialutil.SerialException as e:
			rospy.logwarn("Serial exception when writing into a register. %s", str(e))
		except (minimalmodbus.NoResponseError, minimalmodbus.InvalidResponseError) as e:
			rospy.logwarn("%s.", str(e))
		self.rcu_lock.release()
		return result

	def get_rcu_status(self):
		"""
        Get charging and hardware information from RCU unit
        and store the information to RCUModbusInterface object variable
        """
		self.rcu_lock.acquire()
		# charging status information

		try:
			for i in range(int(_MODBUS_INPUT_REGISTER_NUM / _MODBUS_GROUP_SIZE)):
				addr_start = _MODBUS_GROUP_SIZE * i
				self._regs_input[addr_start:addr_start + _MODBUS_GROUP_SIZE] = self._instrument.read_registers(registeraddress=addr_start,
																												number_of_registers=_MODBUS_GROUP_SIZE,
																												functioncode=_MODBUS_READ_INPUT_REGISTERS)

		except serial.serialutil.SerialException as e:
			rospy.logwarn("Could not read input register. %s", str(e))
			self.charge_status = ChargerState.RCU_NOT_CONNECTED
			self.charge_status_message = translate_charge_status(self.charge_status)
			self.lost_packet_count += 1
		except (minimalmodbus.NoResponseError, minimalmodbus.InvalidResponseError):
			rospy.logdebug("Invalid Modbus response.")
			self.charge_status = ChargerState.RCU_NOT_CONNECTED
			self.charge_status_message = translate_charge_status(self.charge_status)
			self.lost_packet_count += 1
		except minimalmodbus.IllegalRequestError as e:
			rospy.logwarn(f'Could not read input register. {e}')
			self.charge_status = ChargerState.RCU_NOT_CONNECTED
			self.charge_status_message = translate_charge_status(self.charge_status)
			self.lost_packet_count += 1
		else:
			self.lost_packet_count = 0
			charge_status = self._regs_input[_MODBUS_ADDRESS_CHARGE_STATUS]
			self.charge_status = charge_status
			self.charge_status_message = translate_charge_status(charge_status)

			# rcu device id information
			self.rcu_device_id = self._regs_input[_MODBUS_ADDRESS_RCU_DEVICE_ID]

			# RCU real time error
			self.errors = []
			rcu_error_low = self._regs_input[_MODBUS_ADDRESS_RCU_ERROR_LOW]
			if rcu_error_low > 0:
				for error in rcu_error_data_list:
					if (rcu_error_low & error):
						error_msg = translate_error_code(error, 'low')
						self.errors.append(error_msg)

			rcu_error_high = self._regs_input[_MODBUS_ADDRESS_RCU_ERROR_HIGH]
			if not rcu_error_high > 0:
				for error in rcu_error_data_list:
					if (rcu_error_high & error):
						error_msg = translate_error_code(error, 'high')
						self.errors.append(error_msg)
			self.error_code = rcu_error_high << 16 | rcu_error_low

			# Shadow Error
			shadow_error_low = self._regs_input[_MODBUS_ADDRESS_SHADOW_ERROR_LOW]

			# append only new shadow errors to error list
			new_shadow_error_low = (~self._previous_shadow_error_low) & shadow_error_low
			if new_shadow_error_low > 0:
				for error in rcu_error_data_list:
					if (new_shadow_error_low & error):
						error_msg = translate_error_code(error, 'low')
						self.errors.append(error_msg)
			self._previous_shadow_error_low = shadow_error_low

			shadow_error_high = self._regs_input[_MODBUS_ADDRESS_SHADOW_ERROR_HIGH]
			# append only new shadow errors to error list
			new_shadow_error_high = (~self._previous_shadow_error_high) & shadow_error_high
			if new_shadow_error_high > 0:
				for error in rcu_error_data_list:
					if (new_shadow_error_high & error):
						error_msg = translate_error_code(error, 'high')
						self.errors.append(error_msg)
			self._previous_shadow_error_high = shadow_error_high
			self.shadow_error_code = shadow_error_high << 16 | shadow_error_low

			# output current information
			output_current = self._regs_input[_MODBUS_ADDRESS_OUTPUT_CURRENT]
			self.output_current = float(output_current / 128)
			# battery voltage information
			battery_voltage = self._regs_input[_MODBUS_ADDRESS_BATTERY_VOLTAGE]
			self.battery_voltage = float(battery_voltage / 128)

			# RCU Hardware Status variable
			# device temperature information
			self.device_temperature = self._regs_input[_MODBUS_ADDRESS_DEVICE_TEMPERATURE]

			# coil temperature information
			self.coil_temperature = self._regs_input[_MODBUS_ADDRESS_COIL_TEMPERATURE]

			self._range_check_status = self._regs_input[_MODBUS_ADDRESS_READ_RANGE_CHECK_STATUS]

		finally:
			self.rcu_lock.release()

		if self.lost_packet_count >= self.lost_packet_tolerence:
			self.charge_status = ChargerState.RCU_NOT_CONNECTED
			self.charge_status_message = "comm error"
			self.is_connected = False
			rospy.logerr(f'lost {self.lost_packet_count} packets continusly, please check your connection and power up RCU unit')
