# ds5_uwb_remote_control

Use PS5 controller and UWB system to remote control your robot 

Contact me: lhjh0212531@gmail.com.

If you are DIT's member, you can DM me with Slack or FB(Messenger).

### Submodule

ds5_ros: https://github.com/autonohm/ds5_ros

nlink_uwb_tools: https://github.com/DIT-ROBOTICS/nlink_uwb_tools

### Contents
* [Install (On your laptop AND your robot's computer)](#install-on-your-laptop-and-your-robots-computer)
* [UWB setup](#uwb-setup)
    * [NAssistant](#nassistant)
    * [ttyUSB permission check](#ttyusb-permission-check)
* [ds5_ros setup (ONLY on laptop)](#ds5_ros-setup-only-on-laptop)
    * [Installing](#installing)
    * [Device Access](#device-access)
    * [Change pydualsense.py](#change-pydualsense-py)
* [Usage](#usage)
    * [Test Run (roslaunch)](#test-run-roslaunch)
    * [Node Graph](#node-graph)
    * [Topic](#topic)
    * [remote_control](#remote_control)
    * [Write your own controll code and launch file](#write-your-own-controll-code-and-launch-file)

## Install (On your laptop AND your robot's computer)

Install as submodule(recommend):

Under `YOUR_WOEKSPACE/src`
```bash
git submodule add git@github.com:HaoYuLiu0725/ds5_uwb_remote_control.git
git submodule update --init --recursive
```

Under `YOUR_WOEKSPACE/src`
```bash
cd ds5_uwb_remote_control/nlink_uwb_tools/serial
```

Under `YOUR_WOEKSPACE/src/ds5_uwb_remote_control/nlink_uwb_tools/serial`
```bash
make; sudo make install
```

And you need to enter your password
```bash
cd ../../../..
```

Under `YOUR_WOEKSPACE`
```bash
catkin_make --only-pkg-with-deps nlink_parser
source devel/setup.bash 
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
```
You can `catkin_make` again to ensure that doesn't have any problem.

## UWB setup

LinkTrack Datasheet: https://ftp.nooploop.com/software/products/uwb/doc/LinkTrack_Datasheet_V2.2_zh.pdf

LinkTrack Message: https://ftp.nooploop.com/software/products/uwb/doc/NLink_V1.3.pdf

Prepare **two** LinkTrack S module, in DIT Robotics lab you could find 8 of it.

<img src="https://github.com/HaoYuLiu0725/ds5_uwb_remote_control/blob/main/image/LinkTrack_S.png" width="300" height="300">

Donwload and install NAssistant: https://www.nooploop.com/en/download/

In my case, I use Windows system to configure UWB.

### NAssistant

Open NAssistant, and plug in your UWB, use Device Manager(裝置管理員) to check COM port of your UWB.

Make sure choose correct **COM port** and **baud rate(default should be 921600)**, and click **connect**.

<img src="https://github.com/HaoYuLiu0725/ds5_uwb_remote_control/blob/main/image/NAssistant_connection.png" width="680" height="440">

As you connected, the interface should look like one of these two images, the left one is **anchor**, and the right one is **tag**.

If you want to use UWB's built-in IMU on your robot, make sure use **tag** on you robot.

Next, click **setup** button to setup UWB.

<img src="https://github.com/HaoYuLiu0725/ds5_uwb_remote_control/blob/main/image/anchor_and_tag.png" width="800" height="250">

There are several things to setup: **System ID**, **System CH**, **TX Gain**, **Mode Parameters**, and **Role**.

If you want to know more detail, go to read this [datasheet](https://ftp.nooploop.com/software/products/uwb/doc/LinkTrack_Datasheet_V2.2_zh.pdf).

<img src="https://github.com/HaoYuLiu0725/ds5_uwb_remote_control/blob/main/image/UWB_setup.png" width="800" height="200">

**1. System ID:** A variable to distinguish different System identities. Nodes of the same System must use same System ID. Please check other teams System ID, make sure not to conflict.

(I'm using System ID 0, so please ***DON'T use System ID 0***)

**2. System CH:** Nodes of the same System must use same System CH. Different channel have different Centre Frequency, Band, Bandwidth, and Encoder Mode. As we using LinkTrack S module, the 2、3、4、5、8、9 channel are better.

**3. TX Gain:** Nodes of the same System must use same TX Gain. TX Gain have 0 ~ 33.5(dB) range, more TX Gain will give you more range.

**4. Mode Parameters:** Different Mode have different characteristic, **LP_MODE5 is recommended**, you can read [datasheet](https://ftp.nooploop.com/software/products/uwb/doc/LinkTrack_Datasheet_V2.2_zh.pdf) page 15 to know more about Mode Parameters.

**5. Role:** As we are using LP_MODE5, we can use **1 anchor and 1 tag** to communicate, it is recommend to use **tag** on your robot, and use **anchor** on you laptop or fixed beacon, so we can use it for UWB localization, or use it **tag**'s built-in IMU for robot.

**6. ID:** Please **DON'T** change UWB's ID, because it should already have it's own ID label on it's backside.

**7. Others:** NOT necessary and NOT recommend to change, so please **DON'T** change any other thing.

### ttyUSB permission check

If you use Windows to configure UWB like me, and use virtual machine(VMware) to run Ubuntu,

don't forget to unplug UWB connection and reconnect to the virtual machine.

**On Raspberry Pi**, you can create serial port alias (fix USB port name):

(Optional but recommend, ttyUSB depends on plug in order of USB)

Under `YOUR_WOEKSPACE/src/ds5_uwb_remote_control`
```bash
chmod +x ./rename_RPI_USB_ports.sh
sudo ./rename_RPI_USB_ports.sh
```

Check device connected
```bash
ll /dev | grep ttyUSB
```
If I/O issue, make sure USB permission is r/w-able.

try 
```bash
sudo usermod -a -G dialout $USER
```
Add the user into dialout group to get permission permanently on most hosts.

## ds5_ros setup (ONLY on laptop)
### Installing
Install the hidapi.

```bash
sudo apt install libhidapi-dev
```
Install the package from [pypi](https://pypi.org/project/pydualsense/).

```bash
pip install pydualsense
```
### Device Access
To allow linux to access the controller, you will need to set the device permissions:

```bash
sudo chmod -R 777 /dev
```

OR you can add the provided UDEV rule to your rules.d folder to permanently enable this for your system:

```bash
sudo cp YOUR_WOEKSPACE/src/ds5_uwb_remote_control/ds5_ros/udev/99-dualsense.rules /etc/udev/rules.d
```
### Change pydualsense.py
You can use this command to find pydualsense.py:

```bash
find . -name pydualsense.py
```

In pydualsense.py, **add** attribute cable_connection in function init():

```python
def init(self):
    ...
    self.cable_connection = True
    ...
```

**Change** function writeReport() in pydualsense.py to catch the IOError when controller is disconnect to the PC

```python
def writeReport(self, outReport):
    """
    write the report to the device

    Args:
        outReport (list): report to be written to device
    """
    try:
        self.device.write(bytes(outReport))
    except IOError:
        self.cable_connection = False
```
## Usage
### Test Run (roslaunch)

**1. On laptop:** plug in **anchor UWB** and your **PS5 controller**. UWB default is working on ttyUSB0, use `ll /dev | grep ttyUSB` to check UWB connection.

laptop.launch running nodes: ds5ros_node, linktrack0, joy_serialization
```bash
roslaunch remote_control laptop.launch
```
**2. On robot:** plug in **tag UWB**, UWB default is working on USB0 (or ttyUSB1), use `ll /dev | grep USB` to check UWB connection.

robot.launch running nodes: remote_control, linktrack0, joy_deserialization
```bash
roslaunch remote_control robot.launch
```
You can also use your own launch file, just add `<include file="$(find remote_control)/launch/robot.launch"/>` in your launch file.

And don't forget to use rosserial to connect STM32 or Arduino.

### Node Graph

laptop.launch:

<img src="https://github.com/HaoYuLiu0725/ds5_uwb_remote_control/blob/main/image/node_graph_laptop.png" width="1300" height="80">

robot.launch:

<img src="https://github.com/HaoYuLiu0725/ds5_uwb_remote_control/blob/main/image/node_graph_robot.png" width="1300" height="150">

### Topic
**1. laptop.launch:**

(1) `ds5ros_node`:  

subscribe: `/joy/set_feedback`(remap it to `/joy_set_feedback`) [[sensor_msgs/JoyFeedbackArray]](http://docs.ros.org/en/api/sensor_msgs/html/msg/JoyFeedbackArray.html)

publish: `/joy`(remap it to `/nlink0/joy`)[[sensor_msgs/Joy]](http://docs.ros.org/en/api/sensor_msgs/html/msg/Joy.html)

(2) `joy_serialization`(in group`/nlink0`):

subscribe: `/joy`[[sensor_msgs/Joy]](http://docs.ros.org/en/api/sensor_msgs/html/msg/Joy.html)
 
publish: `/serialized_msg`(remap it to `/nlink_linktrack_data_transmission`)[[std_msgs/String]](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/String.html)
        
**2. robot.launch:**

(1) `joy_deserialization`(in group`/nlink1`):
    
subscribe: `/node_frame`(remap it to `/nlink_linktrack_nodeframe0`)[[nlink_parser/LinktrackNodeframe0]]
        
publish: `/ds5_joy`[[sensor_msgs/Joy]](http://docs.ros.org/en/api/sensor_msgs/html/msg/Joy.html)
        
(2) `remote_control`:
        
subscribe: `/ds5_joy`(change it to `/nlink1/ds5_joy` in param)[[sensor_msgs/Joy]](http://docs.ros.org/en/api/sensor_msgs/html/msg/Joy.html)
        
publish: 

`/cmd_vel`[[geometry_msgs/Twist]](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html)

`/arm_goal`[[geometry_msgs/Point]](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Point.html)
                
`/suck`[[std_msgs/Bool]](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html)

### remote_control

**1. param:**

[double]
    
"frequency": default = 50 [Hz], timercallback loop frequency.

"MAX_linear_speed": default = 0.8 [m/s], max speed of robot's linear movement (Vx & Vy).

"MAX_angular_speed": default = 1.0 [rad/s], max speed of robot's angular movement (Omega).

"init_arm_x", "init_arm_y", "init_arm_z": default = 128.0, 17.0, 10.0 [mm], arm's initial position.

"arm_MAX_XYspeed", "arm_MAX_Zspeed": default = 100.0, 100.0 [mm/s], arm's max moving speed.

"X_max", "X_min", "Y_max", "Y_min", "Z_max", "Z_min": default = 516.0, -281.0, 516.0, -516.0, 119.0, -58.0 [mm], arm's movement limitation.

[std::string]
    
"joy_topic": default = `/ds5_joy` [[sensor_msgs/Joy]](http://docs.ros.org/en/api/sensor_msgs/html/msg/Joy.html)
    
"twist_topic": default = `/cmd_vel` [[geometry_msgs/Twist]](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html)
    
"point_topic": default = `/arm_goal` [[geometry_msgs/Point]](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Point.html)
    
"suck_topic": default = `/suck` [[std_msgs/Bool]](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html)

**2. Controller config:**

<img src="https://github.com/HaoYuLiu0725/ds5_uwb_remote_control/blob/main/image/control_config.png" width="800" height="600">

### Write your own controll code and launch file

1. Remember subscribe to `/nlink1/ds5_joy` [[sensor_msgs/Joy]](http://docs.ros.org/en/api/sensor_msgs/html/msg/Joy.html) to get PS5 controller input. (Or remap to any topic you like)

2. Check [ds5_ros controller mapping](https://github.com/autonohm/ds5_ros/tree/e9794bee23f1b6c0af76a50014fd06c813c5132e#controler-mapping).

3. Rember to add the following code in your launch file. And don't forget to use rosserial to connect STM32 or Arduino.
```xml
<group ns="nlink1">
    <node pkg="nlink_parser" type="linktrack" name="linktrack0" output="screen">
        <!-- Testing on laptop -->
        <!-- <param name="port_name" value="/dev/ttyUSB1" />  -->
        <!-- Using on Raspberry Pi 4 -->
        <param name="port_name" value="/dev/USB0"/>
        <param name="baud_rate" value="921600"/>
    </node>
    <node pkg="remote_control" type="joy_deserialization" name="joy_deserialization" output="screen">
        <remap from="node_frame" to="nlink_linktrack_nodeframe0"/>
    </node>
</group>
```
