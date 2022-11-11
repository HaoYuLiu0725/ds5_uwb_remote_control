# ds5_uwb_remote_control

Use PS5 controller and UWB system to remote control your robot 

## Submodule

ds5_ros: https://github.com/autonohm/ds5_ros

nlink_uwb_tools: https://github.com/DIT-ROBOTICS/nlink_uwb_tools

## Install

Under `YOUR_WOEKSPACE/src`
```bash
git clone --recursive https://github.com/HaoYuLiu0725/ds5_uwb_remote_control.git
```
### ds5_ros
#### Installing
Install the hidapi.

```bash
sudo apt install libhidapi-dev
```
Install the package from [pypi](https://pypi.org/project/pydualsense/).

```bash
pip install pydualsense
```
#### Device Access
To allow linux to access the controller, you will need to set the device permissions:

```bash
sudo chmod -R 777 /dev
```

OR you can add the provided UDEV rule to your rules.d folder to permanently enable this for your system:

```bash
sudo cp YOUR_WOEKSPACE/src/ds5_uwb_remote_control/ds5_ros/udev/99-dualsense.rules /etc/udev/rules.d
```
#### Change pydualsense.py

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
### nlink_uwb_tools
#### UWB setup

Prepare **two** LinkTrack S

<img src="/HaoYuLiu0725/ds5_uwb_remote_control/raw/main/image/LinkTrack_S.png">

Donwload the NAssistant: https://www.nooploop.com/en/download/. In my case, I use Windows system to configure UWB
#### ttyUSB permission

Check device connected

`ll /dev | grep ttyUSB`

If I/O issue, make sure USB permission is r/w-able.

try 

`sudo usermod -a -G dialout $USER`

Add the user into dialout group to get permission permanently on most hosts.
