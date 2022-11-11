# ds5_uwb_remote_control

Use PS5 controller and UWB system to remote control your robot 

# Submodule

ds5_ros: https://github.com/autonohm/ds5_ros

nlink_uwb_tools: https://github.com/DIT-ROBOTICS/nlink_uwb_tools

# Install

Under `YOUR_WOEKSPACE/src`
```bash
git clone --recursive https://github.com/HaoYuLiu0725/ds5_uwb_remote_control.git
```

## ds5_ros
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
To allow linux to access the controller, you will need to set the device permissions.

add the provided UDEV rule to your rules.d folder to permanently enable this for your system:

```bash
sudo cp YOUR_WOEKSPACE/src/ds5_uwb_remote_control/ds5_ros/udev/99-dualsense.rules /etc/udev/rules.d
```
## nlink_uwb_tools
