# openvr_ros

This package connects to [SteamVR](https://store.steampowered.com/steamvr) through [OpenVR](https://github.com/ValveSoftware/openvr) SDK and publishes tracking information of all connected devices (i.e. headsets, controllers, and trackers) to ROS.

This package is tested with `Ubuntu 16.04`, `ROS Kinetic`, and `HTC Vive`.

<p align="center">
  <img src="./doc/rviz.gif" width="640">
</p>

For calibration of frame origins and CAD model sizes please watch:

<p align="center">
  <a href="http://www.youtube.com/watch?feature=player_embedded&v=mAlpZEBwjqs
  " target="_blank"><img src="http://img.youtube.com/vi/mAlpZEBwjqs/0.jpg" 
  alt="Calibration of HTC Vive ROS interface with a point cloud sensor" width="640" height="480" border="10" /></a>
</p>

## Installation

* Install [SteamCMD](https://developer.valvesoftware.com/wiki/SteamCMD). For the list of commands for SteamCMD see: [Command_Line_Options](https://developer.valvesoftware.com/wiki/Command_Line_Options).

* Install SteamVR by the following command:

```bash
# AppID of SteamVR on Steam store is 250820: https://steamdb.info/app/250820/
steamcmd +login anonymous +app_update 250820 +quit
```

* Install [udev.rules](https://raw.githubusercontent.com/ValveSoftware/steam-devices/master/60-steam-vr.rules). See [usb device requirements](https://github.com/ValveSoftware/SteamVR-for-Linux/blob/master/README.md#usb-device-requirements) for more details.

```bash
cd
wget https://raw.githubusercontent.com/ValveSoftware/steam-devices/master/60-steam-vr.rules
sudo mv 60-steam-vr.rules /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger
```

* Install [OpenVR](https://github.com/ValveSoftware/openvr):

```bash
git clone https://github.com/ValveSoftware/openvr.git
cd openvr
mkdir build && cd build
cmake ..
make
sudo make install
```

* Clone the `openvr_ros` package into your catkin workspace.

```bash
cd <path_to_your_catkin_workspace>
catkin_make
```

## Usage

* Connect your VR device.
* Run SteamVR, then calibrate the device:

```bash
cd ~/.steam/SteamApps/common/SteamVR/bin
STEAM_RUNTIME=0 ./vrstartup.sh >> /dev/null 2>&1
```

* Run the tracker:

```bash
source <path_to_your_catkin_workspace>/devel/setup.bash

roslaunch openvr_ros start.launch
```

## TODO

- [X] Publish all tracking information for the headset, controllers, and trackers.
- [X] Add 3D models of the headset and controllers/trackers to the rviz visualization.
- [ ] Publish controller states.
- [ ] Publish events, e.g. button press, device connections, standby, etc.


## Issues
Please file issues under the [issues tab](https://github.com/sharif1093/openvr_ros/issues).

## References

* A great tutorial on using OpenVR: https://github.com/osudrl/CassieVrControls/wiki/OpenVR-Quick-Start
* OpenVR wiki: https://github.com/ValveSoftware/openvr/wiki
