# MyActuator RMD X-series ROS 2

Author: [Tobit Flatscher](https://github.com/2b-t) (2024)

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)



## Overview
This repository holds the **URDF models** and [**`ros2_control` integration**](https://control.ros.org/humble/index.html) for the [**MyActuator RMD-X actuator series**](https://www.myactuator.com/rmd-x). The CAD models for the URDF models were obtained from the [offical MyActuator RMD web page](https://www.myactuator.com/downloads-x-series). The hardware interface is based on the [C++ driver that I have written for these actuators](https://github.com/2b-t/myactuator_rmd). You will need to clone both of the repositories to your ROS 2 workspace and [follow these instructions for installing its dependencies](https://github.com/2b-t/myactuator_rmd/blob/main/ReadMe.md):

```bash
$ cd /colcon_ws/src
$ git clone https://github.com/2b-t/myactuator_rmd.git
$ git clone https://github.com/2b-t/myactuator_rmd_ros.git
```

For more information regarding the individual packages inside this repository please refer to their corresponding individual read-mes.

![MyActuator RMD X8-Pro V2](./myactuator_rmd_description/media/X8ProV2.png)
