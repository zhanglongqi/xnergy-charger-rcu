#!/usr/bin/env python3
import rospy
from xnergy_charger_rcu.srv import DummyComm, DummyCommRequest, DummyCommResponse


class RCUDummyAdapter:
	def __init__(self):
		""" Initialize RCU with dummy interface """
		self.is_connected = True
		self.firmware_version_number = -1
		self.battery_voltage = -1
		self.output_current = -1
		self.runtime_voltage_setting = 12.34
		self.runtime_current_setting = 23.45
		self.charge_status = -1
		self.charge_status_message = ''
		self.errors = []
		self.device_temperature = 20
		self.coil_temperature = 21
		self.error_code = 0
		self.shadow_error_code = 0

		self.lost_packet_count = 0
		self.lost_packet_tolerence = 10

		self.range_check_status = 1
		self.set_status_srv = rospy.Service('~set_status', DummyComm, self.set_status)

	def connect(self):
		self.is_connected = True
		self.firmware_version_number = 0xdeadbeef
		return True

	def enable_charge(self):
		return True

	def disable_charge(self):
		return True

	def request_range_check(self) -> bool:
		return True

	def read_range_check_status(self) -> (bool, int):
		return True, self.range_check_status

	def set_status(self, request: DummyCommRequest):
		if 'main_state' in request.name:
			self.charge_status = request.value
		elif 'error_code' in request.name:
			self.error_code = request.value
		elif 'range_check_status' in request.name:
			self.range_check_status = request.value
		else:
			return DummyCommResponse(success=False, message='Unknown parameter')
		return DummyCommResponse(success=True, message='OK')

	def get_rcu_status(self):
		pass
