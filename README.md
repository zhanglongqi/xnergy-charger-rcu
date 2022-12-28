

- [Xnergy RCU ROS driver](#xnergy-rcu-ros-driver)
  - [System Requirements](#system-requirements)
  - [Getting Started](#getting-started)
  - [ROS interfaces](#ros-interfaces)
    - [Config parameters](#config-parameters)
    - [Published Topics](#published-topics)
    - [Services](#services)
    - [Action Server](#action-server)
    - [Charger Action Server](#charger-action-server)
    - [Range Check Action Server](#range-check-action-server)
  - [Connecting RCU](#connecting-rcu)
  - [Others](#others)
    - [Modbus](#modbus)
    - [CANbus](#canbus)
    - [GPIO](#gpio)
  - [Docker](#docker)
  - [Maintainer](#maintainer)
  - [Contributors](#contributors)

----


# Xnergy RCU ROS driver

This package is a ROS driver for Xnergy wireless charger. The ROS driver facilitates communication with the Receiver Unit (RCU) mounted on the robot via Modbus, CANbus, or GPIO.


## System Requirements

This package is tested under these environment setup:

* [Ubuntu 20.04 Focal Fossa](https://wiki.ubuntu.com/FocalFossa/ReleaseNotes) 
* [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
* Python version == 3.8.10
* Python Library:
  * aenum==3.1.5
  * gpiod==1.5.0
  * minimalmodbus==2.0.1
  * pyserial==3.5
  * python-can==3.3.4
  * wrapt==1.13.3


## Getting Started

1. Create catkin workspace and clone repository as a package:

    ```
    mkdir -p catkin_ws/src/xnergy_charger_rcu
    git clone https://github.com/westonrobot/xnergy_charger_rcu catkin_ws/src/xnergy_charger_rcu
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

or with argument

```shell
roslaunch xnergy_charger_rcu xnergy_charger_canbus.launch device:="canNUM" bitrate:="250000"
```

**GPIO**:

```shell
roslaunch xnergy_charger_rcu xnergy_charger_gpio.launch
```

## ROS interfaces

### Config parameters

* **Modbus**:
  * `device` : default value = '/dev/ttyUSB0' (depend on the serial port used)
  * `communication_interface` : value = 'modbus'

* **CANbus**:
  * `device` : default value = 'can0' (depend on the CAN port used)
  * `bitrate` : default value = 250000 (depend on the CANBUS setting in the RCU)
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

<!-- * `~battery_state`(type [`sensor_msgs/BatteryState.msg`](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/BatteryState.html)) : topic to monitor RCU output current, battery voltage, charger voltage -->

* `/diagnostics` : ROS diagnostics message for Xnergy Charger RCU Node for hardware information: **device temperature** and **coil temperature**.

### Services

`xnergy_charger_rcu` ROS node offers `~trigger_charging` service of type `std_srvs/Trigger`. Calling this service will enable charging. This is a nonblocking call, that is charging request is sent to RCU but doesn't wait to finish enable charging procedure. This is the easiest way to start/stop charging:

```shell
rosservice call /xnergy_charger_rcu/start_charging
```

```shell
rosservice call /xnergy_charger_rcu/stop_charging
```

### Action Server
In any large ROS based system, there are cases when someone would like to send a request to a node to perform some task, and also receive a reply to the request. This can currently be achieved via ROS services.

In some cases, however, if the service takes a long time to execute, the user might want the ability to cancel the request during execution or get periodic feedback about how the request is progressing. The actionlib package provides tools to create servers that execute long-running goals that can be preempted. It also provides a client interface in order to send requests to the server.

For details on actionlib please refer to [documentation](http://wiki.ros.org/actionlib). This is the best way to integrate.

Xnergy Charger RCU Node provides the following action servers:
Charge Action Server and Range Check Action Server.


The easiest way to test action server is to use the command line tool from actionlib.
Install `actionlib-tools` if you did not install it before:

```shell
sudo apt install ros-noetic-actionlib-tools
```

### Charger Action Server

Charger Action Server is an `actionlib` server controlling charging at `/xnergy_charger_rcu/charge` of custom type `Charge.action`. 

To test a goal to Charger Action Server, you could use `axclient` from `actionlib`. Run:
```shell
rosrun actionlib_tools axclient.py /xnergy_charger_rcu/charge
```

### Range Check Action Server

`Range Check` is available only for communication interface with Xnergy RCU using `canbus` and `modbus`.

Range Check Action Server is an `actionlib` server used for controlling Xnergy RCU at `/xnergy_charger_rcu/rangecheck` of custom type `RangeCheck.action`. 

To send a goal to Range Check Action Server, you could use `axclient` from `actionlib`. Run:

```shell
rosrun actionlib_tools axclient.py /xnergy_charger_rcu/rangecheck
```

Users or Systems need to wait 5 seconds before sending a new goal to `Range Check` Action Server only when the previous goal has accomplished 
because the Xnergy RCU requires some time to be ready. 

`Range Check` feature is available only for Xnergy RCU with firmware V1.25 onwards.

```shell
|                                      |
|                                      |
|                 goal 1               |
|     +-------------------------->     |
|                                      |
|               feedback 1             |
|     <--------------------------+     |
|               feedback 2             |
|     <--------------------------+     |
|               feedback N             |
|     <--------------------------+     |
|               result 1               |
|     <--------------------------+     |
|                                      |
|                                      |
|         wait for 5 seconds before    |
|           sending a new goal         |
|                                      |
|               goal 2                 |
|     +--------------------------->    |
|                                      |
|                                      |
|                                      |
v                                      v
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
sudo ip link set down can0
# sudo ip link set can0 txqueuelen 1000
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

## Maintainer

* Zhang LongQi

## Contributors

* Jakub Tomášek
* Alex Chua Zhi Hao
* Zhang LongQi
