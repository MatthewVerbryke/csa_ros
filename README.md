# Canonical Software Architecture (CSA) for ROS

## Summary

This repository contains the implementation of a modified version of JPL's Canonical Software Architecture (CSA) format for ROS 1.0 using its Python interface. The CSA module format is intended to increase the modularity of autonomous system control architectures, while also enforcing a consistent and standardized structure. Also include are the rosmsg types for directive-response messages used in inter- and intra-module communication.

**NOTE**: The code in this repository is still very much a work in progress; check back for updates as development continues.

## Recommended OS/Programs

This software was developed and tested in:
- Ubuntu 20.04 LTS
- ROS Noetic
- Python 3.8

## Installation

Clone the repository into your catkin workspace `src` directory. Then build the workspace in the root catkin directory using `catkin_make`.

## Usage

TODO

## Future Work

Next steps include:
- Expand communication protocols that can be utilized, including rosbridge/websockets
- Implementation of an Activity Managment component
- Further improvements to module/component code
- Create a usage guide
- Include an implemented example module

## License

Unless otherwise noted in a file, This program is licensed under the BSD 3-clause license, as presented in the LICENSE file

Program is Copyright (c) University of Cincinnati
