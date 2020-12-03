# nuridgeback_robot

# Overview
This package contains the launch files capable of launching sensors, mobile base, navigation, exploration, localization, and mapping on the Ridgeback.

# Getting Started
On startup the ridgeback will already be running `base.launch` and `laser_slam.launch` via
the `ridgeback.service`.


Steps 1 through 4 are executed on your personal computer.
Step 5 must be executed on the Ridgeback's computer.

1) On your personal computer setup the environment.

The `ROS_IP` maybe be different. This will be assigned to
your computer by the Ridgeback's hotspot.
```
export ROS_MASTER_URI=http://192.168.131.40:11311
export ROS_IP=10.42.0.171
unset ROS_HOSTNAME
```

Edit `/etc/hosts` to contain:
```
192.168.131.40  sawyer.local
```

In order for Sawyer's computer to be able to ping your computer run:
```
sudo ip route add 192.168.131.0/24 via 10.42.0.1
```

To configure the ip route using network manager so you don'y have to run the
command above every time see [Configuring Static Routes Using NMCLI.](https://access.redhat.com/documentation/en-us/red_hat_enterprise_linux/7/html/networking_guide/sec-configuring_static_routes_using_nmcli).

Test whether the packets are send to Sawyer run:
```
traceroute 192.168.131.40
```

You should see that packets were sent to sawyer.local.

#### TODO: provide example output

2) Workspace

#### TODO: add ergodic_exploration and range_mi to rosinstall

Important: make sure to build the install space
```
mkdir -p ~/ridgeback_ws/src
cd ~/ridgeback_ws/src
wstool init .
wstool merge -t . https://github.com/bostoncleek/nu_ridgeback/blob/master/ridgebackws.rosinstall
wstool update -t .
cd ~/ridgeback_ws
catkin init
```
```
catkin build --install -DMCAKE_BUILD_TYPE=Release
```

3) Copy install space to the Ridgeback
```
rsync -av install/ administrator@192.168.131.1:/home/administrator/install
```

### TODO: command to kill the ref nodes
4)


5) Enable the Ridgeback and Sawyer

Make sure non of the E-stop buttons are engaged.
Press the button on the left rear side of the Ridgeback to reset it's E-stop.

ssh into the Ridgeback:
```
ssh administrator@192.168.131.1
```

enable sawyer:
```
cd ~/sawback_ws
source devel/setup.bash
rosrun inter_interface enable_robot.py -e
```


# Launch Files
Each time a modification is made to the package you must build and then copy the
install space to the Ridgeback.

To see how to launch nodes from your personal computer but have them execute on the Ridgeback's
computer see `basic_remote.launch`.

* velodyne.launch:
* bumblebee.launch:
* accessories.launch:
* base.launch:
* basic_remote.launch:
* laser_slam.launch:
* visual_slam.launch:
* move_base.launch:
* navigation.launch:
* exploration.launch:
* Visualization.laucnh
* basic_autonomy.launch:

The launch files primarily used are:
