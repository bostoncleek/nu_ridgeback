# nuridgeback_robot

# Overview
This package contains the launch files capable of launching sensors, mobile base, navigation, exploration, localization, and mapping on the Ridgeback.


# Getting Started
Steps 1 through 4 are executed on your personal computer.
Step 5 and 6 must be executed on the Ridgeback's computer.

1. On your personal computer setup the environment.

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
  command above every time see [Configuring Static Routes Using NMCLI](https://access.redhat.com/documentation/en-us/red_hat_enterprise_linux/7/html/networking_guide/sec-configuring_static_routes_using_nmcli).

  Test whether the packets are send to Sawyer run:
  ```
  traceroute 192.168.131.40
  ```

  You should see packets were sent to sawyer.local.

  ```
  traceroute to 192.168.131.40 (192.168.131.40), 30 hops max, 60 byte packets
   1  ridgeback (10.42.0.1)  1.299 ms  1.270 ms  1.407 ms
   2  sawyer.local (192.168.131.40)  1.458 ms  1.565 ms  1.665 ms

  ```

2. Workspace
  - Important: make sure to build the install space
  - Important: make sure to `source install/setup.bash` in the workspace after building

  ```
  mkdir -p ~/ridgeback_ws/src
  cd ~/ridgeback_ws/src
  wstool init .
  wstool merge -t . https://github.com/bostoncleek/nu_ridgeback/blob/master/nuridgeback.rosinstall
  wstool update -t .
  cd ~/ridgeback_ws
  catkin init
  ```
  ```
  catkin build --install -DMCAKE_BUILD_TYPE=Release
  ```

3. Copy install space to the Ridgeback
  ```
  rsync -av install/ administrator@192.168.131.1:/home/administrator/install
  ```

4. Kill Sawyer's ref state publisher and TFs
  ```
  rosnode kill /robot_ref_publisher
  rosnode kill /ref_base_to_world
  rosnode kill /base_to_world
  ```

5. Stop the `ridgeback.service`
  On boot the Ridgeback is running the `ridgeback.service` which starts `base.launch` and `laser_slam.launch`.

  First, ssh into the Ridgeback:
  ```
  ssh administrator@192.168.131.1
  ```

  To stop this service:
  ```
  systemctl stop ridgeback.service
  ```

  Then setup the environment by running:
  ```
  consawyer
  ```

  The `consawyer` function connects to sawyer by:
  ```
  export ROS_MASTER_URI='http://192.168.131.40:11311'
  export ROS_IP='192.168.131.1'
  unset ROS_HOSTNAME
  ```

6. Enable the Ridgeback and Sawyer

  Make sure non of the E-stop buttons are engaged.
  Press the button on the left rear side of the Ridgeback to reset it's E-stop.

  enable sawyer:
  ```
  cd ~/sawback_ws
  source devel/setup.bash
  rosrun intera_interface enable_robot.py -e
  ```

# Launch Files
Each time a modification is made to the package you must build and then copy the
install space to the Ridgeback.

To see how to launch nodes from your personal computer but have them execute on the Ridgeback's
computer see `basic_remote.launch`.

The two most useful launch files are `base.launch` and `laser_slam.launch`.
The file `base.launch` takes an argument `ridgeback_urdf` which will either load the Sawback URDF or the Ridgeback URDF. By defualt the Sawback URDF is loaded. The file `base.launch` will start odometry and allow you to teleoperate the Ridgeback using the PS4 gamepad. The file `laser_slam.launch` will perform 3D SLAM using the Velodyne lidar. The occupancy map on `/ridgeback/rtabmap/grid_map`, TF from `map` to `base_link`, and the robot's path on `/ridgeback/rtabmap/mapGraph` are provided by RTAB-MAP.

In order to launch `base.launch` and `laser_slam.launch` Sawyer's computer must be on because it is running ROS master.

Once Sawyer has fully booted execute the following from the Ridgeback's computer:
```
ssh administrator@192.168.131.1
cd ~/sawback_ws
source devel/setup.bash
```

```
roslaunch nuridgeback_robot base.launch
roslaunch nuridgeback_robot laser_slam.launch
```

The `robot` argument specifies whether the node will run on the Ridgeback's computer or
your personal computer. Set the argument to `0` to run the node on the Ridgeback's computer and set to `1` to run on your personal computer. By default the argument is set to `0` in `navigation.launch` and `exploration.launch`.

The only nodes that should run on your personal computer should be for visualization.


To start the ROS navigation stack:
```
roslaunch nuridgeback_robot navigation.launch
```

To start the ergodic exploration package using user specified targets:
```
roslaunch nuridgeback_robot exploration.launch
```

To start the ergodic exploration package using user mutual information:
```
roslaunch nuridgeback_robot exploration.launch use_mi:=true
```

To visualize SLAM results:
```
roslaunch nuridgeback_robot visualization.launch viz_slam:=true
```

To visualize navigation results:
```
roslaunch nuridgeback_robot visualization.launch viz_nav:=true
```

To visualize exploration results:
```
roslaunch nuridgeback_robot visualization.launch viz_explore:=true
```

* velodyne.launch: Velodyne lidar sensor launch
* bumblebee.launch: Bumblebee camera sensor launch
* accessories.launch: Velodyne, Hokoyu, and Bumblebee sensor launch
* base.launch: start the Ridgeback base this allows for teleoperation
* basic_remote.launch: sets machine tag
* laser_slam.launch: RTAB-MAP 3D SLAM using Velodyne
* visual_slam.launch: RTAB-MAP 3D SLAM using Bumblebee
* move_base.launch: ROS Navigation using move_base
* navigation.launch: Sets machine tag and launches move_base
* exploration.launch: Ergodic exploration
* visualization.launch: visualize SLAM, navigation, and exploration results
