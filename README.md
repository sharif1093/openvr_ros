# openvr_ros

This package connects to [SteamVR](https://store.steampowered.com/steamvr) through [OpenVR](https://github.com/ValveSoftware/openvr) SDK and publishes tracking information of all connected devices (i.e. headsets, controllers, and trackers) to ROS.

This package is tested with `Ubuntu 16.04`, `ROS Kinetic`, and `HTC Vive`.

<p align="center">
  <img src="./doc/rviz.gif" width="640">
</p>

## Installation

* Install [Steam](https://store.steampowered.com/about/), create an account if you don't have one and connect to it.
* Find SteamVR on the Steam store and install it.
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
* Run SteamVR, calibrate the device.
* Run the tracker:

```bash
source <path_to_your_catkin_workspace>/devel/setup.bash

roslaunch openvr_ros start.launch
```

## TODO

- [X] Publish all tracking information for the headset, controllers, and trackers.
- [ ] Publish controller states.
- [ ] Publish events, e.g. button press, device connections, standby, etc.
- [ ] Add 3d models of the headset and controllers/trackers to the rviz visualization.

## Issues
Please file issues under the [issues tab](https://github.com/sharif1093/openvr_ros/issues).

## References

* A great tutorial on using OpenVR: https://github.com/osudrl/CassieVrControls/wiki/OpenVR-Quick-Start
* OpenVR wiki: https://github.com/ValveSoftware/openvr/wiki
