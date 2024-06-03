# MyActuator RMD X-series Hardware

Author: [Tobit Flatscher](https://github.com/2b-t) (2024)

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)



## Overview
This package holds the [**`ros2_control` integration**](https://control.ros.org/humble/index.html) for the [**MyActuator RMD-X actuator series**](https://www.myactuator.com/rmd-x) in the form of a [hardware component](https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/hardware_components_userdoc.html). The hardware interface is based on the [C++ driver that I have written for these actuators](https://github.com/2b-t/myactuator_rmd).

For using it add the following lines to your URDF refering to the joint of interest `joint_name`:

```xml
<ros2_control name="${some_name}" type="actuator">
  <hardware>
    <plugin>myactuator_rmd_hardware/MyActuatorRmdHardwareInterface</plugin>
    <param name="ifname">${ifname}</param>
    <param name="actuator_id">${actuator_id}</param>
    <param name="torque_constant">${torque_constant}</param>
  </hardware>
  <joint name="${joint_name}">
    <command_interface name="position"/>
    <command_interface name="velocity"/>
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>
```

The `ifname` has to correspond to the name of the CAN interface as shown by `$ ifconfig` (e.g. `can0`) and the `actuator_id` to the ID of the actuator (e.g. `1`). The `torque_constant` is required for controlling the actuator over its effort interface and depends on the actuator type. For more information refer to the examples inside `myactuator_rmd_description`.

