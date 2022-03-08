from xnergy_charger_rcu.msg import ChargerState

absolut_zero_temperature = -273.15


def translate_charge_status(data):
	""" from RCU register address data to charger status(in string) """
	if data == ChargerState.RCU_IDLE:
		return 'idle'
	elif data in (ChargerState.RCU_HANDSHAKE_0, ChargerState.RCU_HANDSHAKE_1, ChargerState.RCU_HANDSHAKE_2, ChargerState.RCU_HANDSHAKE_3):
		return 'handshake'
	elif data == ChargerState.RCU_CHARGING:
		return 'charging'
	elif data in (ChargerState.RCU_STOP, ChargerState.RCU_SEC_RAMP_DOWN, ChargerState.RCU_SEC_RESTART_DELAY):
		return 'stopping'
	elif data in (ChargerState.RCU_SEC_WAIT_TIE_TO_BOOTLOADER, ChargerState.RCU_SEC_PRE_READY, ChargerState.RCU_SEC_DEBUG_READY, ChargerState.RCU_SEC_DEBUG_UNPAIR,
					ChargerState.RCU_SEC_DEBUG_PAIRED, ChargerState.RCU_SEC_DEBUG_SEC_ENABLE_PRI, ChargerState.RCU_SEC_DEBUG_ERROR, ChargerState.RCU_SEC_MANUAL_CHARGE):
		return 'XNERGY_RESERVED'
	elif data == ChargerState.RCU_ERROR:
		return 'error'
	elif data == ChargerState.RCU_NOT_CONNECTED:
		return 'comm_error'
	else:
		return 'unknown status'


def translate_error_code(data: int, lohi: bool):
	"""
    Xnergy unit stores 2 error status at high value and low value
    From RCU register address data to error status (in string)
    """
	if lohi == 'high':
		if data == 0x0002:
			return 'Foreign object detected'
		elif data == 0x0001:
			return 'TPU return error'
		elif data == 0x0010:
			return ' NTC TEMPERATURE ERROR'
		else:
			return 'Unknown error'
	elif lohi == 'low':
		if data == 0x8000:
			return 'One of fan is stuck'
		elif data == 0x4000:
			return 'RCU over temperature detected'
		elif data == 0x0200:
			return 'Transmitter encounters error during charging'
		elif data == 0x0100:
			return 'Communication lost during charging'
		elif data == 0x0080:
			return 'Gap between transmitter pad and RCU is too close'
		elif data == 0x0040:
			return 'Gap between transmitter pad and RCU is too far'
		elif data == 0x0020:
			return 'Communication lost during coupling estimation'
		elif data == 0x0010:
			return 'Handshake timeout'
		elif data == 0x0008:
			return 'Authentication timeout'
		elif data == 0x0004:
			return 'Communication loss after pairing'
		elif data == 0x0002:
			return 'Pairing timeout'
		elif data == 0x0001:
			return 'Flash memory is not set properly'
		else:
			return 'Unknown error'
