/**
 * \file myactuator_rmd_hardware_interface.hpp
 * \mainpage
 *    ROS 2 Control hardware interface for MyActuator RMD-X series actuators based on CAN
 *    See https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#ifndef MYACTUATOR_RMD_HARDWARE__MYACTUATOR_RMD_HARDWARE_INTERFACE
#define MYACTUATOR_RMD_HARDWARE__MYACTUATOR_RMD_HARDWARE_INTERFACE
#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <myactuator_rmd/driver/can_driver.hpp>
#include <myactuator_rmd/actuator_interface.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "myactuator_rmd_hardware/visibility_control.hpp"


namespace myactuator_rmd_hardware {

  /**\class MyActuatorRmdHardwareInterface
   * \brief
   *    Hardware interface for the MyActuator RMD X-series actuators based on CAN
  */
  class MyActuatorRmdHardwareInterface: public hardware_interface::ActuatorInterface {
    public:
      MyActuatorRmdHardwareInterface() = default;
      MyActuatorRmdHardwareInterface(MyActuatorRmdHardwareInterface const&) = default;
      MyActuatorRmdHardwareInterface& operator = (MyActuatorRmdHardwareInterface const&) = default;
      MyActuatorRmdHardwareInterface(MyActuatorRmdHardwareInterface&&) = default;
      MyActuatorRmdHardwareInterface& operator = (MyActuatorRmdHardwareInterface&&) = default;

      /**\fn ~MyActuatorRmdHardwareInterface
       * \brief
       *    Class destructor
       *    Makes sure the actuator is shut down cleanly
      */
      ~MyActuatorRmdHardwareInterface();

      /**\fn on_configure
       * \brief
       *    Callback function for configure transition
       *    Sets up communication with the hardware
       * 
       * \param[in] previous_state
       *    Previous state that we are transitioning from
       * \return
       *    Value indicating success, error or failure of the callback
      */
      CallbackReturn on_configure(rclcpp_lifecycle::State const& previous_state) override;

      /**\fn on_cleanup
       * \brief
       *    Callback function for cleanup transition
       *    Gracefully shuts down the actuator
       * 
       * \param[in] previous_state
       *    Previous state that we are transitioning from
       * \return
       *    Value indicating success, error or failure of the callback
      */
      CallbackReturn on_cleanup(rclcpp_lifecycle::State const& previous_state) override;
      
      /**\fn on_shutdown
       * \brief
       *    Callback function for shutdown transition
       *    Gracefully shuts down the actuator
       * 
       * \param[in] previous_state
       *    Previous state that we are transitioning from
       * \return
       *    Value indicating success, error or failure of the callback
      */
      CallbackReturn on_shutdown(rclcpp_lifecycle::State const& previous_state) override;

      /**\fn on_activate
       * \brief
       *    Callback function for activate transition
       *    Enables hardware power
       * 
       * \param[in] previous_state
       *    Previous state that we are transitioning from
       * \return
       *    Value indicating success, error or failure of the callback
      */
      CallbackReturn on_activate(rclcpp_lifecycle::State const& previous_state) override;

      /**\fn on_deactivate
       * \brief
       *    Callback function for deactivate transition
       *    Disables hardware power
       * 
       * \param[in] previous_state
       *    Previous state that we are transitioning from
       * \return
       *    Value indicating success, error or failure of the callback
      */
      CallbackReturn on_deactivate(rclcpp_lifecycle::State const& previous_state) override;
      
      /**\fn on_error
       * \brief
       *    Callback function for error transition
       *    Resets actuator
       * 
       * \param[in] previous_state
       *    Previous state that we are transitioning from
       * \return
       *    Value indicating success, error or failure of the callback
      */
      CallbackReturn on_error(rclcpp_lifecycle::State const& previous_state) override;

      /**\fn on_init
       * \brief
       *    Callback function for init transition
       *    Initializes all member variables
       * 
       * \param[in] info
       *    Hardware info containing the information from the URDF
       * \return
       *    Value indicating success, error or failure of the callback
      */
      CallbackReturn on_init(hardware_interface::HardwareInfo const& info) override;

      /**\fn export_state_interfaces
       * \brief
       *    Export the state interfaces that the actuator exposes
       * 
       * \return
       *    The state interfaces the actuator exposes
      */
      std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

      /**\fn export_command_interfaces
       * \brief
       *    Export the command interfaces that the actuator exposes
       * 
       * \return
       *    The command interfaces the actuator exposes
      */
      std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

      /**\fn prepare_command_mode_switch
       * \brief
       *    Prepare the switch of the command mode of the actuator
       * 
       * \param[in] start_interfaces
       *    Interfaces to be started
       * \param[in] stop_interfaces
       *    Interfaces to be stopped
       * \return
       *    Value indicating success, error or failure of the callback
      */
      hardware_interface::return_type prepare_command_mode_switch(std::vector<std::string> const& start_interfaces,
        std::vector<std::string> const& stop_interfaces) override;

      /**\fn perform_command_mode_switch
       * \brief
       *    Switch of the command mode of the actuator
       * 
       * \param[in] start_interfaces
       *    Interfaces to be started
       * \param[in] stop_interfaces
       *    Interfaces to be stopped
       * \return
       *    Value indicating success, error or failure of the callback
      */
      hardware_interface::return_type perform_command_mode_switch(std::vector<std::string> const& start_interfaces,
        std::vector<std::string> const& stop_interfaces) override;
      
      /**\fn read
       * \brief
       *    Read the current state of the actuator
       * 
       * \param[in] time
       *    The current time
       * \param[in] period
       *    The time passed since the last read
       * \return
       *    Value indicating success, error or failure of the callback
      */
      hardware_interface::return_type read(rclcpp::Time const& time, rclcpp::Duration const& period) override;

      /**\fn write
       * \brief
       *    Write the next command to the actuator
       * 
       * \param[in] time
       *    The current time
       * \param[in] period
       *    The time passed since the last write
       * \return
       *    Value indicating success, error or failure of the callback
      */
      hardware_interface::return_type write(rclcpp::Time const& time, rclcpp::Duration const& period) override;
  
    protected:      
      /**\fn getLogger
       * \brief
       *    Get the logger used for console output
       * 
       * \return
       *    The logger
      */
      static rclcpp::Logger getLogger();

      /**\fn commandThread
       * \brief
       *    The asynchronous command thread used to communicate with the hardware
       *    that performs a combined read and write
       * 
       * \param[in] cycle_time
       *    The cycle time that the asynchronous thread should run at
      */
      void commandThread(std::chrono::milliseconds const& cycle_time);

      /**\fn startCommandThread
       * \brief
       *    Start the asynchronous command thread used to communicate with the hardware
       * 
       * \param[in] cycle_time
       *    The cycle time that the asynchronous thread should run at
       * \return
       *    Boolean variable indicating successful start of the async thread or failure
      */
      [[nodiscard]]
      bool startCommandThread(std::chrono::milliseconds const& cycle_time);
      
      /**\fn stopCommandThread
       * \brief
       *    Stop the asynchronous command thread used to communicate with the hardware
      */
      void stopCommandThread();
      
      std::string ifname_;
      std::uint32_t actuator_id_;
      double torque_constant_;
      double max_velocity_;

      // Buffers only used by the main thread
      double position_state_;
      double velocity_state_;
      double effort_state_;
      double position_command_;
      double velocity_command_;
      double effort_command_;

      // The command thread reads and writes from the actuator cyclically
      std::thread command_thread_;
      std::chrono::milliseconds cycle_time_;
      // Never accessed by both threads at the same time
      std::unique_ptr<myactuator_rmd::CanDriver> driver_;
      std::unique_ptr<myactuator_rmd::ActuatorInterface> actuator_interface_;
      // Shared between the two threads
      std::atomic<bool> stop_command_thread_;
      std::atomic<myactuator_rmd::Feedback> feedback_;
      std::atomic<double> command_thread_position_;
      std::atomic<double> command_thread_velocity_;
      std::atomic<double> command_thread_effort_;
      std::atomic<bool> position_interface_running_;
      std::atomic<bool> velocity_interface_running_;
      std::atomic<bool> effort_interface_running_;
  };

}  // namespace myactuator_rmd_hardware

#endif  // MYACTUATOR_RMD_HARDWARE__MYACTUATOR_RMD_HARDWARE_INTERFACE
