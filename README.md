# nu_ridgeback
- [Overview](#Overview) </br>
- [Packages](#Packages) </br>
- [Demos](#Demos) </br>
  - [Mobile Manipulation](#Mobile-Manipulation) </br>
  - [Ergodic Exploration](#Ergodic-Exploration) </br>

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

# Demos
## Mobile Manipulation
To see the custom pick and place code using MoveIt! and [Grasp Pose Detection](https://github.com/atenpas/gpd) see [sawback_manipulation](https://github.com/bostoncleek/sawback/tree/master/sawback_manipulation). To see the full mobile manipulation sequence see this [video](https://youtu.be/iLyqu9EoNtY).

<p align="center">
  <img src="/nuridgeback_robot/media/ridgeback_pick.gif" width="300" height="300"/>
  <img src="/nuridgeback_robot/media/pick_rviz.gif" width="300" height="300"/>
</p>

## Ergodic Exploration
Checkout the [ergodic_exploration](https://github.com/bostoncleek/ergodic_exploration) package. This package is capable of allowing the user to specify the information density target distribution represented as Gaussians or using mutual information.

The user specified targets are shown in the left (bottom left corner) demo overlaying the occupancy grid in yellow. The mutual information map is show on the right (upper left corner) demo. Full videos of both exploration using preset [target distribution video](https://youtu.be/SmzaeUUY6QQ) and the [mutual information video](https://youtu.be/iYFPkeTlLi4) are shown in real time.

<p align="center">
  <img src="/nuridgeback_robot/media/two_targets.gif" width="300" height="300"/>
  <img src="/nuridgeback_robot/media/mi_atrium.gif" width="300" height="300"/>
</p>
