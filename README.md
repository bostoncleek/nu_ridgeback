# nu_ridgeback

# Overview
A custom repository for Northwestern University's Ridgeback. The Ridgeback is equipped with a Velodyne lidar, Bumblebee depth camera, 2 Hokoyu UST10LX lidars and a Sawyer manipulator.

On startup the ridgeback will already be running `base.launch` and `laser_slam.launch` via
the `ridgeback.service`.

Ridgeback computer:
```
username: administrator
password: nu-ridgeback
```

Ridgeback hotspot:
```
name: Hostspot
password: ridgeback
```

<p align="center">
  <img src="/nuridgeback_robot/media/nurb1.jpg" width="400" height="400"/>
</p>


# Packages
* [nuridgeback_bringup](https://github.com/bostoncleek/nu_ridgeback/tree/master/nuridgeback_bringup): networking, configuration, and ROS Noetic
* [nuridgeback_description](https://github.com/bostoncleek/nu_ridgeback/tree/master/nuridgeback_description): urdf and meshes
* [nuridgeback_gazebo](https://github.com/bostoncleek/nu_ridgeback/tree/master/nuridgeback_gazebo): gazebo simulation
* [nuridgeback_robot](https://github.com/bostoncleek/nu_ridgeback/tree/master/nuridgeback_robot): launch sensors, mobile base, navigation, exploration, localization, and mapping

For manipulation see the [sawback](https://github.com/bostoncleek/sawback) package.
