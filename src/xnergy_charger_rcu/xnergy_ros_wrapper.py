#!/usr/bin/env python3

import diagnostic_msgs
import diagnostic_updater
import rospy
from std_srvs.srv import Trigger, TriggerResponse

# from sensor_msgs.msg import BatteryState
from xnergy_charger_rcu.msg import ChargerState, RangeCheckFeedback
from xnergy_charger_rcu.utils import translate_charge_status


class XnergyChargerROSWrapper:

	def __init__(self):
		"""
        Init Communication and define ROS publishers
        """
		# define ros publishers
		self.rcu_status_pub = rospy.Publisher("~rcu_status", ChargerState, queue_size=10)
		# self.rcu_battery_pub = rospy.Publisher("~battery_state", BatteryState, queue_size=10)

		# init connection
		self.comm_interface = rospy.get_param("~communication_interface", "modbus")
		self.init_connection(self.comm_interface)

		# RCU diagnostic_updater
		self.diag_updater = diagnostic_updater.Updater()
		self.diag_updater.setHardwareID(rospy.get_name())

		if 'modbus' == self.comm_interface or 'dummy' == self.comm_interface:
			self.diag_updater.add("Device Temperature", self.check_device_temperature)
			self.diag_updater.add("Coil Temperature", self.check_coil_temperature)
		else:
			rospy.logwarn(f'coil temerature and device temperature are not available for communication interface {self.comm_interface}')

		# RCU Status update
		self.status_update()

		self.start_charging_srv = rospy.Service('~start_charging', Trigger, self.start_charging)

		self.stop_charging_srv = rospy.Service('~stop_charging', Trigger, self.stop_charging)

	def init_connection(self, interface):
		"""
        Init comunication with the interface and check whether it is connected
        """
		if interface == "modbus":
			from xnergy_charger_rcu.rcu_modbus_adapter import RCUModbusAdapter

			# baudrate = rospy.get_param("~baudrate", 9600)
			baudrate = 9600
			port = rospy.get_param("~device", "/dev/ttyUSB0")
			rospy.loginfo(f'serial port device: {port=}, baudrate: {baudrate}')
			self.rcu = RCUModbusAdapter(port=port, baudrate=baudrate)

		elif interface == "canbus":
			from xnergy_charger_rcu.rcu_canbus_adapter import RCUCANbusAdapter
			port = rospy.get_param("~device", "can0")
			node_id = rospy.get_param("~node_id", 10)
			rospy.loginfo(f'CAN bus device: {port=}, CANOpen {node_id=}')
			self.rcu = RCUCANbusAdapter(port=port, node_id=node_id)

		elif interface == "gpio":
			from xnergy_charger_rcu.rcu_gpio_adapter import RCUGPIOAdapter
			charger_control = {}
			charger_control['chip'] = rospy.get_param("~charger_control_chip", "/dev/gpiochip0")
			charger_control['line'] = rospy.get_param("~charger_control_line", 1)
			rospy.loginfo("GPIO device for charger control: " + charger_control['chip'] + " Line: " + str(charger_control['line']))

			charger_status = {}
			charger_status['chip'] = rospy.get_param("~charger_status_chip", "/dev/gpiochip0")
			charger_status['line'] = rospy.get_param("~charger_status_line", 2)
			rospy.loginfo("GPIO device for charger status: " + charger_status['chip'] + " Line: " + str(charger_status['line']))

			self.rcu = RCUGPIOAdapter(charger_control=charger_control, charger_status=charger_status)
		elif interface == "dummy":
			from xnergy_charger_rcu.rcu_dummy_adapter import RCUDummyAdapter
			rospy.loginfo(msg="Dummy RCU adapter")
			self.rcu = RCUDummyAdapter()
		else:
			rospy.logerr("XNERGY:: incorrect ros parameter input : " + interface)
			rospy.signal_shutdown("Incorrect ros parameter")
			exit()

		timer = 0
		while not self.rcu.connect():
			rospy.logerr("XNERGY:: " + interface + ":: Unable to connect to the RCU")
			rospy.logerr('check your interface settings and try again')
			msg = ChargerState()
			msg.header.stamp = rospy.Time.now()
			msg.state = msg.RCU_NOT_CONNECTED
			msg.message = str("RCU not connected")
			self.rcu_status_pub.publish(msg)
			timer += 10
			rospy.sleep(10)
			if timer >= 60 or rospy.is_shutdown():
				rospy.logerr("XNERGY:: " + interface + ":: Status: RCU NOT CONNECTED")
				rospy.signal_shutdown("Unable to connect to the RCU")
				exit()

		rospy.loginfo("XNERGY:: " + interface + ":: Connected to the RCU")
		self.rcu.get_rcu_status()

		if (self.rcu.firmware_version_number > 0):
			firmware_version_number = f'{self.rcu.firmware_version_number:04X}'
		else:
			firmware_version_number = "Unknown"

		if (self.rcu.runtime_voltage_setting >= 0):
			runtime_voltage_setting = str(self.rcu.runtime_voltage_setting) + " V"
		else:
			runtime_voltage_setting = "Unknown"

		if (self.rcu.runtime_current_setting >= 0):
			runtime_current_setting = str(self.rcu.runtime_current_setting) + " A"
		else:
			runtime_current_setting = "Unknown"

		rospy.loginfo("RCU Firmware Version: " + firmware_version_number)
		rospy.loginfo("RCU Runtime Voltage Setting: " + runtime_voltage_setting)
		rospy.loginfo("RCU Runtime Current Setting: " + runtime_current_setting)

	@property
	def is_connected(self):
		return self.rcu.is_connected

	def status_update(self):
		"""
        Get RCU Charge Status, Battery Status, Error Status from RCU
        and publish to ROS topics
        """
		if self.is_connected:
			self.rcu.get_rcu_status()
		else:
			rospy.logwarn('try to connect back')
			self.rcu.connect()

		# real time error checking
		# if len(self.rcu.errors) > 0:
		# 	for error_msg in self.rcu.errors:
		# 		rospy.logerr("Xnergy RCU ERROR DETECTED: " + error_msg)

		# publish RCU status
		msg = ChargerState()
		msg.header.stamp = rospy.Time.now()
		msg.state = self.rcu.charge_status
		msg.runtime_voltage_setting = self.rcu.runtime_voltage_setting
		msg.runtime_current_setting = self.rcu.runtime_current_setting
		msg.battery_voltage = self.rcu.battery_voltage
		msg.output_current = self.rcu.output_current
		msg.error_code = self.rcu.error_code
		msg.shadow_error_code = self.rcu.shadow_error_code
		msg.message = str(self.rcu.charge_status_message)
		self.rcu_status = msg
		self.rcu_status_pub.publish(msg)

		# publish Battery Status
		# msg = BatteryState()
		# msg.header.stamp = rospy.Time.now()
		# msg.voltage = self.rcu.battery_voltage
		# msg.current = self.rcu.output_current
		# battery_full = self.rcu.runtime_voltage_setting if self.rcu.runtime_voltage_setting > 0 else 1
		# msg.percentage = 1 - (battery_full - self.rcu.battery_voltage) / battery_full
		# if self.rcu.charge_status_message == 'charging':
		# 	msg.power_supply_status = 1
		# elif self.rcu.charge_status_message == 'stop':
		# 	msg.power_supply_status = 2
		# elif self.rcu.charge_status_message == 'idle':
		# 	msg.power_supply_status = 3
		# else:
		# 	msg.power_supply_status = 0

		# self.rcu_battery_pub.publish(msg)

		# diagnostic_updater
		if self.rcu.is_connected:
			self.diag_updater.update()

	def charging_switch(self, goal_command):
		"""
        Function that allow ChargerActionServer to send command directly to RCU unit
        """
		if goal_command:
			return self.rcu.enable_charge()
		else:
			return self.rcu.disable_charge()

	def pre_range_check(self) -> (bool, str):
		"""
		check the RCU status first before sending the command

		"""
		ready = True
		message = 'OK'
		if not self.rcu.is_connected:
			ready = False
			message = "RCU not connected: please check the communication cable with RCU"
		elif self.comm_interface == 'gpio':
			ready = False
			message = 'range check is not supported on GPIO interface'
		elif self.rcu.charge_status == ChargerState.RCU_CHARGING:
			ready = False
			message = 'RCU is Charging: charger is wireless charging operation'
		elif self.rcu.charge_status != ChargerState.RCU_IDLE:
			ready = False
			message = f'Error: main state {self.rcu.charge_status}, error code 0x{self.rcu.error_code:08X}, shadow error code 0x{self.rcu.shadow_error_code:08X}'
		else:
			ready = True
			message = 'OK'
		return ready, message

	def request_range_check(self) -> (bool, int, str):
		if not self.is_connected:
			error_msg = "RCU not connected: please check the communication cable with RCU"
			rospy.logerr(error_msg)
			return False, 0, error_msg

		rcu_status = self.rcu.charge_status
		if not (ChargerState.RCU_IDLE == rcu_status):
			error_msg = (f'request_range_check is not supported when RCU is not in idle state.\n'
							f'the RCU state is expected as RCU_IDLE(0), but got {translate_charge_status(rcu_status)}({rcu_status})')
			rospy.logerr(error_msg)
			return False, rcu_status, error_msg

		result = self.rcu.request_range_check()
		message = ''
		if result:
			rospy.loginfo("XNERGY:: Range Check Requested")
			message = 'range check request sent'
		else:
			rospy.logerr("XNERGY:: failed to request range check")
			message = 'failed to request range check'
		return result, result, message

	def read_range_check_status(self) -> (bool, int, str):
		'''
		read the range check status from RCU

		Args:N/A
		Returns:Tuple(COMMAND_STATUS,RANGECHECK_STATUS,MESSAGE)
			COMMAND_STATUS: True if the command is sent without error
							False if the command is not sent successfully
			RANGECHECK_STATUS: the range check status from RCU
			MESSAGE: convert the range check status to string
					0	: range check in progress
					1	: charger in range
					2	: charger out of range
					240	: error, error code is {0xXXXXXXXX}
		'''
		if not self.is_connected:
			error_msg = "RCU not connected: please check the communication cable with RCU"
			rospy.logerr(error_msg)
			return True, RangeCheckFeedback.RANGE_CHECKING_NOT_APPLICABLE, error_msg

		valid, status = self.rcu.read_range_check_status()

		message = ''
		if valid:
			success = True
			if 0 == status:
				message = 'range check in progress'
			elif 1 == status:
				message = 'charger in range'
			elif 2 == status:
				message = 'charger out of range'
			elif 240 == status:
				message = f'error, error code is {self.rcu.shadow_error_code}'
			else:
				message = 'unknown'
			rospy.loginfo(f"XNERGY:: Range Check Status: {status}, {message}")
		else:
			success = False
			message = 'failed'
			rospy.logerr("XNERGY:: Range Check Status: no ack from RCU")

		return success, status, message

	def check_device_temperature(self, stat):
		"""
        Get RCU Device Temperature and publish to diagnostics topics
        """
		device_temperature = self.rcu.device_temperature
		if device_temperature <= 95:
			stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "Normal: " + str(device_temperature) + " in Celsius")
		else:
			stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Overheat: " + str(device_temperature) + " in Celsius")

		return stat

	def check_coil_temperature(self, stat):
		"""
        Get RCU coil Temperature and publish to diagnostics topics
        """

		coil_temperature = self.rcu.coil_temperature
		if coil_temperature <= 95:
			stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "Normal: " + str(coil_temperature) + " in Celsius")
		else:
			stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "RCU Coil Temperature", level=2, message="Overheat: " + str(coil_temperature) + " in Celsius")
		return stat

	def start_charging(self, request):
		"""
        `start_charging` service callback.
        """
		if not self.rcu.is_connected:
			error_msg = "RCU not connected: please check the communication cable with RCU"
			rospy.logerr(error_msg)
			return TriggerResponse(success=False, message=error_msg)
		elif not (ChargerState.RCU_IDLE == self.rcu.charge_status):
			error_msg = (f'start_charging is not supported when RCU is not in idle state.\n'
							f'the RCU state is expected as RCU_IDLE(0), '
							f'but got {translate_charge_status(self.rcu.charge_status)}({self.rcu.charge_status})')
			rospy.logerr(error_msg)
			return TriggerResponse(success=False, message=error_msg)

		rospy.loginfo("start charging.")
		result = self.charging_switch(True)
		return TriggerResponse(success=result, message="command sent, start charging")

	def stop_charging(self, request):
		"""
        `stop_charging` service callback.
        """
		if not self.rcu.is_connected:
			error_msg = "RCU not connected: please check the communication cable with RCU"
			rospy.logerr(error_msg)
			return TriggerResponse(success=False, message=error_msg)
		elif not (ChargerState.RCU_CHARGING == self.rcu.charge_status):
			error_msg = (f'stop_charging is not supported when RCU is not in RCU_CHARGING state.\n'
							f'the RCU state is expected as RCU_CHARGING(5), '
							f'but got {translate_charge_status(self.rcu.charge_status)}({self.rcu.charge_status})')
			rospy.logerr(error_msg)
			return TriggerResponse(success=False, message=error_msg)

		rospy.loginfo("stop charging.")
		result = self.charging_switch(False)
		return TriggerResponse(success=result, message="command sent, stop charging")
