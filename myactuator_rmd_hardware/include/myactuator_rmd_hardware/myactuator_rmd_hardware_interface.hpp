/**
 * \file myactuator_rmd_hardware_interface.hpp
 * \mainpage
 *    ROS 2 Control hardware interface for MyActuator RMD-X series actuators
 *    See https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#ifndef MYACTUATOR_RMD_HARDWARE__MYACTUATOR_RMD_HARDWARE_INTERFACE
#define MYACTUATOR_RMD_HARDWARE__MYACTUATOR_RMD_HARDWARE_INTERFACE
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <myactuator_rmd/actuator_interface.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "myactuator_rmd_hardware/visibility_control.hpp"


namespace myactuator_rmd_hardware {

  class MyActuatorRmdHardwareInterface: public hardware_interface::ActuatorInterface {
    public:
      MyActuatorRmdHardwareInterface();

      virtual ~MyActuatorRmdHardwareInterface();

      // Set-up communication with the hardware, make sure it can be activated
      CallbackReturn on_configure(rclcpp_lifecycle::State const& previous_state) override;
      
      CallbackReturn on_cleanup(rclcpp_lifecycle::State const& previous_state) override;
      
      // Gracefully shutdown hardware
      CallbackReturn on_shutdown(rclcpp_lifecycle::State const& previous_state) override;

      // Enable hardware power
      CallbackReturn on_activate(rclcpp_lifecycle::State const& previous_state) override;

      // Disable hardware power
      CallbackReturn on_deactivate(rclcpp_lifecycle::State const& previous_state) override;
      
      // Handle errors
      CallbackReturn on_error(rclcpp_lifecycle::State const& previous_state) override;

      // Initialize all member variables and process info parameters
      CallbackReturn on_init(hardware_interface::HardwareInfo const& info) override;

      // Export interfaces
      std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

      std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

      hardware_interface::return_type prepare_command_mode_switch(std::vector<std::string> const& start_interfaces,
        std::vector<std::string> const& stop_interfaces) override;

      hardware_interface::return_type perform_command_mode_switch(std::vector<std::string> const& start_interfaces,
        std::vector<std::string> const& stop_interfaces) override;
      
      // Read the state of the actuator
      hardware_interface::return_type read(rclcpp::Time const& time, rclcpp::Duration const& period) override;

      // Write the command to the actuator
      hardware_interface::return_type write(rclcpp::Time const& time, rclcpp::Duration const& period) override;
  
    protected:
      // Add callback for tuning PID gains through parameters

      double hw_joint_command_;
      double hw_joint_state_;
      std::shared_ptr<myactuator_rmd::ActuatorInterface> actuator_interface_;
  };

}  // namespace myactuator_rmd_hardware

#endif  // MYACTUATOR_RMD_HARDWARE__MYACTUATOR_RMD_HARDWARE_INTERFACE
