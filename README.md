# nu_ridgeback

# Overview
A custom repository for Northwestern University's Ridgeback. The Ridgeback is equipped with a Velodyne lidar, Bumblebee depth camera, 2 Hokoyu UST10LX lidars and a Sawyer manipulator.

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

To configure the Ridgeback from the ground up see [nuridgeback_bringup](https://github.com/bostoncleek/nu_ridgeback/tree/master/nuridgeback_bringup). To get started launching nodes see
[nuridgeback_robot](https://github.com/bostoncleek/nu_ridgeback/tree/master/nuridgeback_robot).

<p align="center">
  <img src="/nuridgeback_robot/media/nurb1.jpg" width="400" height="400"/>
</p>


# Packages
- [nuridgeback_bringup](https://github.com/bostoncleek/nu_ridgeback/tree/master/nuridgeback_bringup): networking, configuration, and ROS Noetic
- [nuridgeback_description](https://github.com/bostoncleek/nu_ridgeback/tree/master/nuridgeback_description): urdf and meshes
- [nuridgeback_gazebo](https://github.com/bostoncleek/nu_ridgeback/tree/master/nuridgeback_gazebo): gazebo simulation
- [nuridgeback_robot](https://github.com/bostoncleek/nu_ridgeback/tree/master/nuridgeback_robot): launch sensors, mobile base, navigation, exploration, localization, and mapping

For manipulation see the [sawback](https://github.com/bostoncleek/sawback) package.
