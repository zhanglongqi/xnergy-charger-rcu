#!/usr/bin/env python3

import rospy
import serial
import actionlib
import diagnostic_msgs
import diagnostic_updater
from sensor_msgs.msg import BatteryState
from xnergy_charger_rcu.msg import ChargerState
from std_srvs.srv import Trigger, TriggerResponse


class XnergyChargerROSWrapper:
    def __init__(self):
        """
        Init Communication and define ROS publishers
        """
        # define ros publishers
        self.rcu_status_pub = rospy.Publisher(
            "~rcu_status", ChargerState, queue_size=10)
        self.rcu_battery_pub = rospy.Publisher(
            "~battery_state", BatteryState, queue_size=10)
        self.diag_updater = diagnostic_updater.Updater()

        # init connection
        self.comm_interface = rospy.get_param(
            "~communication_interface", "modbus")
        self.init_connection(self.comm_interface)

        # RCU Status update
        self.status_update()
        # RCU diagnostic_updater
        self.diag_updater.add("Device Temperature",
                              self.check_device_temperature)
        self.diag_updater.add("Coil Temperature", self.check_coil_temperature)
        self.diag_updater.setHardwareID(rospy.get_name())
        self.start_charging_srv = rospy.Service(
            '~start_charging', Trigger, self.start_charging)

        self.stop_charging_srv = rospy.Service(
            '~stop_charging', Trigger, self.stop_charging)

    def init_connection(self, interface):
        """
        Init comunication with the interface and check whether it is connected
        """
        if interface == "modbus":
            from xnergy_charger_rcu.rcu_modbus_adapter import RCUModbusAdapter
            baudrate = rospy.get_param("~baudrate", 9600)
            port = rospy.get_param("~device", "/dev/ttyUSB0")
            print("Device: " + port)
            self.rcu = RCUModbusAdapter(port=port, baudrate=baudrate)

        elif interface == "canbus":
            from xnergy_charger_rcu.rcu_canbus_adapter import RCUCANbusAdapter
            port = rospy.get_param("~device", "can0")
            rospy.loginfo("Interface: " + port)
            self.rcu = RCUCANbusAdapter(port=port)

        elif interface == "gpio":
            from xnergy_charger_rcu.rcu_gpio_adapter import RCUGPIOAdapter
            charger_control = {}
            charger_control['chip'] = rospy.get_param(
                "~charger_control_chip", "/dev/gpiochip0")
            charger_control['line'] = rospy.get_param("~charger_control_line", 1)
            rospy.loginfo("GPIO device for charger control: " + charger_control['chip'] + " Line: " + str(charger_control['line']))

            charger_status = {}
            charger_status['chip'] = rospy.get_param(
                "~charger_status_chip", "/dev/gpiochip0")
            charger_status['line'] = rospy.get_param("~charger_status_line", 2)
            rospy.loginfo("GPIO device for charger status: " + charger_status['chip'] + " Line: " + str(charger_status['line']))

            self.rcu = RCUGPIOAdapter(charger_control=charger_control, charger_status=charger_status)

        else:
            rospy.logerr("XNERGY:: incorrect ros parameter input : "+interface)
            rospy.signal_shutdown("Incorrect ros parameter")
            exit()

        timer = 0
        while not self.rcu.connect():
            rospy.logerr("XNERGY:: "+interface +
                         ":: Unable to connect to the RCU")
            msg = ChargerState()
            msg.header.stamp = rospy.Time.now()
            msg.state = msg.RCU_NOT_CONNECTED
            msg.message = str("RCU not connected")
            self.rcu_status_pub.publish(msg)
            timer += 1
            rospy.sleep(1)
            if timer >= 60 or rospy.is_shutdown():
                rospy.logerr("XNERGY:: "+interface +
                             ":: Status: RCU NOT CONNECTED")
                rospy.signal_shutdown(
                    "Unable to connect to the RCU")
                exit()

        rospy.loginfo("XNERGY:: " + interface +
                      ":: Connected to the RCU")
        self.rcu.get_rcu_status()

        if (self.rcu.firmware_version_number > 0):
            firmware_version_number = f'{self.rcu.firmware_version_number:04X}'
        else:
            firmware_version_number = "Unknown"

        if (self.rcu.runtime_voltage_setting >= 0):
            runtime_voltage_setting = str(
                self.rcu.runtime_voltage_setting) + " V"
        else:
            runtime_voltage_setting = "Unknown"

        if (self.rcu.runtime_current_setting >= 0):
            runtime_current_setting = str(
                self.rcu.runtime_current_setting) + " A"
        else:
            runtime_current_setting = "Unknown"

        rospy.loginfo("RCU Firmware Version: " + firmware_version_number)
        rospy.loginfo("RCU Runtime Voltage Setting: " +
                      runtime_voltage_setting)
        rospy.loginfo("RCU Runtime Current Setting: " +
                      runtime_current_setting)

    def status_update(self):
        """
        Get RCU Charge Status, Battery Status, Error Status from RCU
        and publish to ROS topics
        """
        self.rcu.get_rcu_status()

        # real time error checking
        if len(self.rcu.errors) > 0:
            for error_msg in self.rcu.errors:
                rospy.logerr("Xnergy RCU ERROR DETECTED: "+error_msg)

        # publish RCU status
        msg = ChargerState()
        msg.header.stamp = rospy.Time.now()
        msg.state = self.rcu.charge_status
        msg.message = str(self.rcu.charge_status_message)
        self.rcu_status = msg
        self.rcu_status_pub.publish(msg)

        # publish Battery Status
        msg = BatteryState()
        msg.header.stamp = rospy.Time.now()
        msg.voltage = self.rcu.battery_voltage
        msg.current = self.rcu.output_current
        battery_full = self.rcu.runtime_voltage_setting if self.rcu.runtime_voltage_setting > 0 else 1
        msg.percentage = 1 - \
            (battery_full - self.rcu.battery_voltage) / battery_full
        if self.rcu.charge_status_message == 'charging':
            msg.power_supply_status = 1
        elif self.rcu.charge_status_message == 'stop':
            msg.power_supply_status = 2
        elif self.rcu.charge_status_message == 'idle':
            msg.power_supply_status = 3
        else:
            msg.power_supply_status = 0

        self.rcu_battery_pub.publish(msg)

        # diagnostic_updater
        if self.rcu.is_connected:
            self.diag_updater.update()

    def send_rcu_command(self, goal_command):
        """
        Function that allow ChargerActionServer to send command directly to RCU unit
        """
        if goal_command == True:
            return self.rcu.enable_charge()
        else:
            return self.rcu.disable_charge()

    def check_device_temperature(self, stat):
        """
        Get RCU Device Temperature and publish to diagnostics topics
        """
        device_temperature = self.rcu.device_temperature
        if device_temperature <= 95:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK,
                         "Normal: "+str(device_temperature)+" in Celsius")
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR,
                         "Overheat: "+str(device_temperature)+" in Celsius")

        return stat

    def check_coil_temperature(self, stat):
        """
        Get RCU coil Temperature and publish to diagnostics topics
        """

        coil_temperature = self.rcu.coil_temperature
        if coil_temperature <= 95:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK,
                         "Normal: "+str(coil_temperature)+" in Celsius")
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "RCU Coil Temperature",
                         level=2, message="Overheat: "+str(coil_temperature)+" in Celsius")
        return stat

    def start_charging(self, request):
        """
        `start_charging` service callback.
        """
        rospy.loginfo("Enabling charging.")
        result = self.send_rcu_command(True)
        return TriggerResponse(success=result)

    def stop_charging(self, request):
        """
        `stop_charging` service callback.
        """
        rospy.loginfo("stop charging.")
        result = self.send_rcu_command(False)
        return TriggerResponse(success=result)
