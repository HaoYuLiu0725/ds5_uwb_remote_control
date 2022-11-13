# ds5_uwb_remote_control

Use PS5 controller and UWB system to remote control your robot 

### Submodule

ds5_ros: https://github.com/autonohm/ds5_ros

nlink_uwb_tools: https://github.com/DIT-ROBOTICS/nlink_uwb_tools

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
You can `catkin_make` again to ensure that doesn't have any problem

## UWB setup

LinkTrack Datasheet: https://ftp.nooploop.com/software/products/uwb/doc/LinkTrack_Datasheet_V2.2_zh.pdf

LinkTrack Message: https://ftp.nooploop.com/software/products/uwb/doc/NLink_V1.3.pdf

Prepare **two** LinkTrack S module, in DIT Robotics lab you could find 8 of it.

<img src="https://github.com/HaoYuLiu0725/ds5_uwb_remote_control/blob/main/image/LinkTrack_S.png" width="300" height="300">

Donwload and install NAssistant: https://www.nooploop.com/en/download/

In my case, I use Windows system to configure UWB

### ttyUSB permission check

If you use Windows to configure UWB like me, and use virtual machine(VMware) to run Ubuntu,

don't forget to unplug UWB connection and reconnect to the virtual machine.

Create serial port alias (fix USB port name):

(Optional but recommend, ttyUSB depends on plug in order of USB)
```bash
chmod +x ./rename_RPI_USB_ports.sh
sudo ./rename_RPI_USB_ports.sh
```
Check device connected
```bash
ll /dev | grep USB
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

In pydualsense.py, add attribute cable_connection in function init():

```python
def init(self):
    ...
    self.cable_connection = True
    ...
```

Change function writeReport() in pydualsense.py to catch the IOError when controller is disconnect to the PC

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

