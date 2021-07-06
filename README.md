# Xnergy RCU ROS driver

This package is a ROS driver for Xnergy wireless charger. The ROS driver facilitates communication with the Receiver Unit (RCU) mounted on the robot via Modbus, CANbus, or GPIO.

----

## Table of Contents

1. [System requirements](#system-requirements)
2. [Getting started](#Getting-started)
3. [ROS interfaces](#ros-interfaces)
    * [Config parameters](#config-parameters)
    * [Published topics](#published-topics)
    * [Services](#services)
    * [Action server](#chargeractionserver)
4. [Connecting RCU](#connecting-RCU)
5. [Others](#others)
6. [Contributors](#contributors)

----

## System Requirements

This package is tested under these environment setup:

* Ubuntu 18.04 Bionic Beaver + [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
* Python version 3.7.*
* Python Library Dependencies:
  * minimalmodbus == 1.0.2
  * pyserial == 3.4
  * python-can == 3.3.4
  * python3-gpiod

## Getting Started

1. Create catkin workspace and clone repository as a package:

    ```shell
    mkdir -p catkin_ws/src/xnergy_charger_rcu
    git clone https://gitlab.com/zhanglongqi/xnergy-charger-rcu.git catkin_ws/src/xnergy_charger_rcu
    ```

2. Install Python dependencies and build:

    ```shell
    pip3 install -r catkin_ws/src/xnergy_charger_rcu/requirements.txt
    cd catkin_ws
    catkin_make
    source devel/setup.bash
    ```

3. To start up a Xnergy Charger RCU Node, run the following commands:

**Modbus**:

```shell
roslaunch xnergy_charger_rcu xnergy_charger_modbus.launch
```

or with argument

```shell
roslaunch xnergy_charger_rcu xnergy_charger_modbus.launch device:="/dev/ttyRCU"
```

**CANbus**:

```shell
roslaunch xnergy_charger_rcu xnergy_charger_canbus.launch
```

**GPIO**:

```shell
roslaunch xnergy_charger_rcu xnergy_charger_gpio.launch
```

## ROS interfaces

### Config parameters

* **Modbus**:
  * `device` : default value = '/dev/ttyUSB0' (depend on the serial port used)
  * `modbus_baudrate` : value = 9600 (Do **NOT** change for Xnergy charger)
  * `communication_interface` : value = 'modbus'

* **CANbus**:
  * `device` : default value = 'can0' (depend on the CAN port used)
  * `communication_interface` : value = 'canbus'

* **GPIO**:
  * `charger_control_chip` : default value = '/dev/gpiochip0' (depend on the gpio used for charger control)
  * `charger_control_line` : default value = 1 (gpio line to start/stop charging, depend on the gpio connection to gpio control pin)

  * `charger_status_chip`: default value = '/dev/gpiochip0' (depend on the gpio used for charger status)
  * `charger_status_line` : default value = 2 (gpio line to read charger status, depend on the gpio connection to gpio status pin)

  * `communication_interface` : value = 'gpio'

### Published Topics

The Xnergy Charger RCU Node publishes data to following topics:

* `~rcu_status`(type `ChargerState`): topic to monitor RCU state (idle, wait RF, Handshake, Paired, Pre-run, charging, stop, restart delay, debug mode, error).

* `~battery_state`(type [`sensor_msgs/BatteryState.msg`](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/BatteryState.html)) : topic to monitor RCU output current, battery voltage, charger voltage

* `/diagnostics` : ROS diagnostics message for Xnergy Charger RCU Node for hardware information: **device temperature** and **coil temperature**.

### Services

`xnergy_charger_rcu` ROS node offers `~trigger_charging` service of type `std_srvs/Trigger`. Calling this service will enable charging. This is a nonblocking call, that is charging request is sent to RCU but doesn't wait to finish enable charging procedure. This is the easiest way to start/stop charging:

```shell
rosservice call /xnergy_charger_rcu/start_charging
```

```shell
rosservice call /xnergy_charger_rcu/stop_charging
```

### ChargerActionServer

ChargerActionServer is `actionlib` server controlling charging at `/xnergy_charger_rcu/charge/` of custom type `Charge.action`. For details on actionlib please refer to [documentation](http://wiki.ros.org/actionlib). This is the best way to integrate  

This action server will handle control by listening for incoming goals on the action name .

To send a goal to `ChargerActionServer`, you coulcould use axclient from `actionlib`. Run:

```shell
rosrun actionlib axclient.py /xnergy_charger_rcu/charge
```

## Connecting RCU

Please refer to Xnergy manual for instructions to safely connect your RCU via Modbus (Port A), CANbus (Port A and B), or GPIO (Port B).

## Others

### Modbus

To find out which USB port is connected, use:

```shell
dmesg | grep tty
```

To control the RCU unit over serial port, you need to give proper privileges to the user:

```shell
sudo usermod -a -G dialout $USER
```

or run:

```shell
sudo chmod 666 /dev/ttyUSB0
```

To list serial devices, run:

```shell
tail -f /var/log/syslog
```

### CANbus

To list can interfaces with debugging information

```shell
ip link show
```

Configuring and Enabling the SocketCAN Interface: To enable CAN interface with RCU unit, run:

```shell
sudo ip link set can0 type can bitrate 250000
sudo ip link set up can0
```

### GPIO

Install `gpiod`

```shell
sudo apt install gpiod
```

Show which gpio you can use

```shell
sudo gpioinfo
```

## Docker

Repository contains `Dockerfile` to build docker image with setup environment for

Build docker image:

```shell
cd <path_to_xnergy_charger_rcu_package>
docker build -t xnergy_charger_rcu:latest .
```

Run docker image for Modbus:

```shell
docker run -it --rm -e ROS_MASTER_URI="http://localhost:11311" -e ROS_IP="127.0.0.1" --device=/dev/ttyRCU xnergy_charger_rcu:latest roslaunch xnergy_charger_rcu xnergy_charger_modbus.launch device:=/dev/ttyRCU
```

## Contributors

* Jakub Tomášek
* Alex Chua Zhi Hao
* Zhang LongQi
