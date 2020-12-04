# nuridgeback_bringup

# Overview
Here are instructions on how to bring the Ridgeback up on ROS Noetic.

# Networking
These instructions show you how to setup the network using network manager.

## IP Address
- Hokoyu front: 192.168.131.20
- Hokoyu rear: 192.168.131.21
- Velodyne lidar: 192.168.131.22
- Sawyer: 192.168.131.40
- br0 (Ridgeback Computer): 192.168.131.1
- Ridgeback MCU: 192.168.131.2
- Hostspot (Ridgeback hotspot): 10.42.0.1

## Network Interfaces
The Ridgeback has the following interfaces: `enp3s0`, `eno1`, and `wlp2s0`. The bridge network `br0`
connect `enp3s0` and `eno1`. The Ridgeback's MCU is connected to `enp3s0`. There is a network switch that connects the Velodyne, both Hokoyus, and Sawyer to eno1. The wireless interface is `wlp2s0` which will allow you to connect you personal computer to the Ridgeback via the Hostspot.

To create the bridge:
```
nmcli con add ifname br0 type bridge con-name br0
```

Next add `enp3s0`, `eno1` to `br0`:
```
nmcli con add type bridge-slave ifname enp3s0 master br0
nmcli con add type bridge-slave ifname eno1 master br0
```

Finally bring up the bridge:
```
nmcli con up br0
```

Verify `br0` is correct:
```
nmcli device
```

The output should show:
```
DEVICE          TYPE      STATE         CONNECTION          
br0             bridge    connected     bridge-br0          
eno1            ethernet  connected     bridge-slave-eno1   
enp3s0          ethernet  connected     bridge-slave-enp3s0

```

## Wireless Hostpost
The hotspot will allow you to connect your personal computer to the Ridgeback. The commands bellow will create a wireless hotspot using `wlp2s0` named Hostpost and password ridgeback.

```
nmcli con add type wifi ifname wlp2s0 con-name Hostspot autoconnect yes ssid Hostspot
nmcli con modify Hostspot 802-11-wireless.mode ap 802-11-wireless.band bg ipv4.method shared
nmcli con modify Hostspot wifi-sec.key-mgmt wpa-psk
nmcli con modify Hostspot wifi-sec.psk "ridgeback"
nmcli con up Hostspot
```

Verify the hotspot is up:
```
nmcli device
```

The output should show:
```
DEVICE          TYPE      STATE         CONNECTION          
wlp2s0          wifi      connected     Hostspot            
```

You will now be able to ssh into the Ridgeback and you should be able to ping all the IP address listed at the top.

## Wireless Gamepad Controller
Use network manager to enable bluetooth and pair the gamepad.  

## Can-bus
To setup the CAN-bus see [robot_upstart](https://github.com/Crowdedlight/robot_upstart). This allows the Ridgeback's computer to communicate with the MCU.

# Udev Rules
Place the udev rules from `/udev` in `/etc/udev/rules.d`. To load and trigger the udev rules
execute:

```
udevadm control --reload-rules && udevadm trigger
```

# Workspace
Currently there are three catkin workspaces chained together on the Ridgeback.
The workspaces are `sawback_ws`, `ridgeback_ws`, and `rtabmap_ws`.

However, this is not necessary. The reason for the separate workspaces is as follows. The `sawback_ws` is used to reduce compile time during development. The `ridgeback_ws` contains all the required packages for using the Ridgeback and Sawyer. The last workspace `rtabmap_ws` only contains RTAB-MAP because I was not able to build it using `catkin build`.

I recommend creating only the `ridgeback_ws`, and `rtabmap_ws` workspaces. Unless if you like to build your workspaces using `catkin_make` instead combine them into one.

First create the `rtabmap_ws` workspace. Follow their [instructions](https://github.com/introlab/rtabmap_ros) on building the package.


To create the `ridgeback_ws`:
```
mkdir -p ~/ridgeback_ws/src
cd ~/ridgeback_ws/src
wstool init .
wstool merge -t . https://github.com/bostoncleek/nu_ridgeback/blob/master/nuridgeback.rosinstall
wstool update -t .
```

Extend the workspace
```
cd ~/ridgeback_ws
catkin init
catkin config --extend=/home/administrator/ridgeback_ws/devel
catkin build -DCMAKE_BUILD_TYPE=Release
```

Verify the extension is correct:
```
echo $CMAKE_PREFIX_PATH
```

The expected output:
```
/home/administrator/ridgeback_ws/devel:/home/administrator/rtabmap_ws/devel:/opt/ros/noetic
```

# Service
- IMPORTANT: The service can only start correctly if the Sawyer's computer has booted. Because of this reason I do not think it makes sense to use the `ridgeback.service`. Instead I found it easier
to manually launch them.

The `ridgeback.service` file in `/service` allows the Ridgeback to launch `base.launch` and
`laser_slam.launch` on startup. This service executes the `nuridgeback_startup` script located in `/scripts`.

Place the `ridgeback.service` in `/etc/systemd/system/` on the Ridgeback's computer.

To enable the service:
```
systemctl enable ridgeback.service
```

To start or stop the service:
```
systemctl start/stop ridgeback.service
```

To check the status:
```
systemctl status ridgeback.service
```

# Calibrate the Bumblebee Camera
Requires [FlyCapture SDK](https://www.flir.com/products/flycapture-sdk/)

```
rosrun camera_calibration cameracalibrator.py --approximate 0.1 --size 8x6 --square 0.025 right:=/camera/right/image_raw left:=/camera/left/image_raw right_camera:=/camera/right left_camera:=/camera/left
```

The calibration file from the `camera_calibration` package is located in `$HOME/.ros/camera_info`.

The camera's serial number is `19363403`. To retrieve camera information run:
```
rosrun pointgrey_camera_driver list_cameras
```
