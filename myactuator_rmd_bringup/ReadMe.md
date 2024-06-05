# MyActuator RMD X-series Bring-up

Author: [Tobit Flatscher](https://github.com/2b-t) (2024)

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)



## Overview
This package contains the **ROS 2 bring-up and configuration files** for the [MyActuator RMD X-series actuators](https://www.myactuator.com/rmd-xplanetary-motor).

For **launching the actuator in simulation** with `ros2_control` simply launch the following launch file specifying the actuator:

```bash
$ ros2 launch myactuator_rmd_bringup myactuator_rmd_control.launch.py actuator:=X8ProV2 simulation:=true
```

Additionally to RViz and Gazebo this should open up a GUI for the controller manager as well as another GUI for the joint trajectory controller. In the latter you can activate the `joint_trajectory_controller` and move the actuator with it.

Alternatively you can switch to the effort, position or velocity controller with the controller manager GUI and publish set-points either to the `/effort_controller/commands`, `/position_controller/commands` or `/velocity_controller/commands` topic with:

```bash
$ ros2 topic pub /effort_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.5]}"
```

![Gazebo preview](./media/gazebo-preview.png)
