#!/usr/bin/env python3

import rospy
import serial
import minimalmodbus
from minimalmodbus import Instrument, MODE_RTU
from xnergy_charger_rcu.msg import ChargerState
from threading import Lock
minimalmodbus.CLOSE_PORT_AFTER_EACH_CALL = True
# MODBUS function code
_MODBUS_READ_SINGLE_COIL = int("0x01", 16)
_MODBUS_READ_SINGLE_DISCRETE = int("0x02", 16)
_MODBUS_READ_HOLDING_REGISTERS = int("0x03", 16)
_MODBUS_READ_INPUT_REGISTERS = int("0x04", 16)
_MODBUS_WRITE_SINGLE_REGISTER = int("0x06", 16)

# Xnergy RCU Modbus Address
_MODBUS_NUM_REGISTERS = 100
# Modbus type: coil(read/write)
_MODBUS_ADDRESS_ENABLE_CHARGING = 0
# Modbus type: input register (read only)
_MODBUS_ADDRESS_CHARGE_STATUS = 51
# Modbus type: holding register (read/write)
_MODBUS_ADDRESS_RCU_DEVICE_ID = 1
# Modbus type: input register (read only)
_MODBUS_ADDRESS_FIRMWARE_VERSION_NUMBER = 99
# Modbus type: holding register (read/write)
_MODBUS_ADDRESS_RUNTIME_VOLTAGE_SETTING = 8
# Modbus type: holding register (read/write)
_MODBUS_ADDRESS_RUNTIME_CURRENT_SETTING = 9
# Modbus type: input register (read only)
_MODBUS_ADDRESS_DEVICE_TEMPERATURE = 36
# Modbus type: input register (read only)
_MODBUS_ADDRESS_COIL_TEMPERATURE = 37
# Modbus type: input register (read only)
_MODBUS_ADDRESS_OUTPUT_CURRENT = 31
# Modbus type: input register (read only)
_MODBUS_ADDRESS_BATTERY_VOLTAGE = 32
# Modbus type: input register (read only)
_MODBUS_ADDRESS_RCU_ERROR_LOW = 52
# Modbus type: input register (read only)
_MODBUS_ADDRESS_RCU_ERROR_HIGH = 53
# Modbus type: input register (read only)
_MODBUS_ADDRESS_SHADOW_ERROR_LOW = 0
# Modbus type: input register (read only)
_MODBUS_ADDRESS_SHADOW_ERROR_HIGH = 1


# this should be ros parameter or arg, will delete later
rcu_error_data_list = [1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 16384, 32768]


def translate_charge_status(data):
    """ from RCU register address data to charger status(in string) """
    if data == ChargerState.RCU_IDLE:
        return 'idle'
    elif data == ChargerState.RCU_RESERVE_0:
        return 'Handshake'
    elif data == ChargerState.RCU_HANDSHAKE_0:
        return 'Handshake'
    elif data == ChargerState.RCU_HANDSHAKE_1:
        return 'Handshake'
    elif data == ChargerState.RCU_HANDSHAKE_2:
        return 'Handshake'
    elif data == ChargerState.RCU_CHARGING:
        return 'charging'
    elif data == ChargerState.RCU_STOP:
        return 'stop'
    elif data == ChargerState.RCU_RESERVE_1:
        return 'RCU_RESERVE_1'
    elif data == ChargerState.RCU_RESERVE_2:
        return 'RCU_RESERVE_2'
    elif data == ChargerState.RCU_ERROR:
        return 'error'


def translate_error_code(data, lohi):
    """ 
    Xnergy unit stores 2 error status at high value and low value
    From RCU register address data to error status (in string)
    """
    if lohi == 'high':
        if data == 2:
            return 'Foreign object detected'
        elif data == 1:
            return 'Wrong header message from transmitter'
        else:
            return 'Unknown error'
    elif lohi == 'low':
        if data == 32768:
            return 'One of fan is stuck'
        elif data == 16384:
            return 'RCU over temperature detected'
        elif data == 512:
            return 'Transmitter encounters error during charging'
        elif data == 256:
            return 'Communication lost during charging'
        elif data == 128:
            return 'Gap between transmitter pad and RCU is too close'
        elif data == 64:
            return 'Gap between transmitter pad and RCU is too far'
        elif data == 32:
            return 'Communication lost during coupling estimation'
        elif data == 16:
            return 'Handshake timeout'
        elif data == 8:
            return 'Authentication timeout'
        elif data == 4:
            return 'Communication loss after pairing'
        elif data == 2:
            return 'Pairing timeout'
        elif data == 1:
            return 'Flash memory is not set properly'
        else:
            return 'Unknown error'

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
        self.runtime_voltage_setting = -1
        self.runtime_current_setting = -1
        self.charge_status = -1
        self.charge_status_message = 'None'
        self.rcu_lock = Lock()
        self._baudrate = baudrate
        self._slave_addr = slave_addr
        self._port = port
        self.errors = []
        self._previous_shadow_error_high = 0
        self._previous_shadow_error_low = 0

    def connect(self):
        try:
            self._instrument = Instrument(
                port=self._port, mode=MODE_RTU, slaveaddress=self._slave_addr)
            self._instrument.serial.baudrate = self._baudrate
            self._instrument.serial.timeout = 1
            self._instrument.mode = MODE_RTU

            # RCU Hardware Setting

            # Read RCU Hardware Setting
            # firmware version information
            firmware_version_number_reg = self._instrument.read_registers(registeraddress=_MODBUS_ADDRESS_FIRMWARE_VERSION_NUMBER, number_of_registers=1,
                                                                          functioncode=_MODBUS_READ_INPUT_REGISTERS)
            self.firmware_version_number = firmware_version_number_reg[0]

            # runtime voltage setting information
            runtime_voltage_setting_reg = self._instrument.read_registers(registeraddress=_MODBUS_ADDRESS_RUNTIME_VOLTAGE_SETTING, number_of_registers=1,
                                                                          functioncode=_MODBUS_READ_HOLDING_REGISTERS)
            self.runtime_voltage_setting = float(runtime_voltage_setting_reg[0]>>7)

            # runtime currnet setting information
            runtime_current_setting_reg = self._instrument.read_registers(registeraddress=_MODBUS_ADDRESS_RUNTIME_CURRENT_SETTING, number_of_registers=1,
                                                                          functioncode=_MODBUS_READ_HOLDING_REGISTERS)
            self.runtime_current_setting = float(runtime_current_setting_reg[0]>>7)
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

    def write_bit(self, register_address, input_value):
        """ Sending command to RCU unit with threading lock to prevent parallel access to interface """
        self.rcu_lock.acquire()
        result = False
        try:
            self._instrument.write_bit(
                registeraddress=register_address, value=input_value)
            result = True
        except serial.serialutil.SerialException as e:
            rospy.logwarn(
                "Serial exception when writing into a register. %s", str(e))
        except (minimalmodbus.NoResponseError, minimalmodbus.InvalidResponseError) as e:
            rospy.logwarn("%s.", str(e))
        self.rcu_lock.release()
        return result

    def reset_errors(self):
        self._previous_shadow_error_low = 0
        self._previous_shadow_error_high = 0

    def get_rcu_status(self):
        """ 
        Get charging and hardware information from RCU unit
        and store the information to RCUModbusInterface object variable
        """
        self.rcu_lock.acquire()
        # charging status information

        try:
            data = self._instrument.read_registers(registeraddress=0, number_of_registers=_MODBUS_NUM_REGISTERS,
                                                   functioncode=_MODBUS_READ_INPUT_REGISTERS)
        except serial.serialutil.SerialException as e:
            rospy.logwarn("Could not read holding register. %s", str(e))
            self.charge_status = ChargerState.RCU_NOT_CONNECTED
            self.charge_status_message = translate_charge_status(
                self.charge_status)
            self.rcu_lock.release()
            return
        except (minimalmodbus.NoResponseError, minimalmodbus.InvalidResponseError) as e:
            rospy.logdebug("Invalid Modbus response.")
            self.charge_status = ChargerState.RCU_NOT_CONNECTED
            self.charge_status_message = translate_charge_status(
                self.charge_status)
            self.rcu_lock.release()
            return
        except:
            rospy.logwarn("Could not read holding register.")
            self.charge_status = ChargerState.RCU_NOT_CONNECTED
            self.charge_status_message = translate_charge_status(
                self.charge_status)
            self.rcu_lock.release()
            return

        charge_status = data[_MODBUS_ADDRESS_CHARGE_STATUS]
        self.charge_status = charge_status
        self.charge_status_message = translate_charge_status(charge_status)

        # rcu device id information
        self.rcu_device_id = data[_MODBUS_ADDRESS_RCU_DEVICE_ID]

        # RCU real time error
        self.errors = []
        rcu_error_low = data[_MODBUS_ADDRESS_RCU_ERROR_LOW]
        if rcu_error_low > 0:
            for error in rcu_error_data_list:
                if (rcu_error_low & error):
                    error_msg = translate_error_code(error, 'low')
                    self.errors.append(error_msg)

        rcu_error_high = data[_MODBUS_ADDRESS_RCU_ERROR_HIGH]
        if not rcu_error_high > 0:
            for error in rcu_error_data_list:
                if (rcu_error_high & error):
                    error_msg = translate_error_code(error, 'high')
                    self.errors.append(error_msg)

        # Shadow Error
        shadow_error_low = data[_MODBUS_ADDRESS_SHADOW_ERROR_LOW]

        # append only new shadow errors to error list
        new_shadow_error_low = (~self._previous_shadow_error_low ) & shadow_error_low
        if new_shadow_error_low > 0:
            for error in rcu_error_data_list:
                if (new_shadow_error_low & error):
                    error_msg = translate_error_code(error, 'low')
                    self.errors.append(error_msg)
        self._previous_shadow_error_low = shadow_error_low

        shadow_error_high = data[_MODBUS_ADDRESS_SHADOW_ERROR_HIGH]
        # append only new shadow errors to error list
        new_shadow_error_high = (~self._previous_shadow_error_high ) & shadow_error_high
        if new_shadow_error_high > 0:
            for error in rcu_error_data_list:
                if (new_shadow_error_high & error):
                    error_msg = translate_error_code(error, 'high')
                    self.errors.append(error_msg)
        self._previous_shadow_error_high = shadow_error_high


        # output current information
        output_current = data[_MODBUS_ADDRESS_OUTPUT_CURRENT]
        self.output_current = float(output_current>>7)
        # battery voltage information
        battery_voltage = data[_MODBUS_ADDRESS_BATTERY_VOLTAGE]
        self.battery_voltage = float(battery_voltage>>7)

        # RCU Hardware Status variable
        # device temperature information
        self.device_temperature = data[_MODBUS_ADDRESS_DEVICE_TEMPERATURE]

        # coil temperature information
        self.coil_temperature = data[_MODBUS_ADDRESS_COIL_TEMPERATURE]
        self.rcu_lock.release()
