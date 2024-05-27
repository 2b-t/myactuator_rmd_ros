#include "myactuator_rmd_hardware/myactuator_rmd_hardware_interface.hpp"

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


namespace myactuator_rmd_hardware {

  MyActuatorRmdHardwareInterface::MyActuatorRmdHardwareInterface() {
    return;
  }

  MyActuatorRmdHardwareInterface::~MyActuatorRmdHardwareInterface() {
    return;
  }

  MyActuatorRmdHardwareInterface::CallbackReturn
  MyActuatorRmdHardwareInterface::on_configure(rclcpp_lifecycle::State const& previous_state) {
    return CallbackReturn::SUCCESS;
  }
      
  MyActuatorRmdHardwareInterface::CallbackReturn
  MyActuatorRmdHardwareInterface::on_cleanup(rclcpp_lifecycle::State const& previous_state) {
    return CallbackReturn::SUCCESS;
  }
  
  MyActuatorRmdHardwareInterface::CallbackReturn
  MyActuatorRmdHardwareInterface::on_shutdown(rclcpp_lifecycle::State const& previous_state) {
    return CallbackReturn::SUCCESS;
  }

  MyActuatorRmdHardwareInterface::CallbackReturn
  MyActuatorRmdHardwareInterface::on_activate(rclcpp_lifecycle::State const& previous_state) {
    return CallbackReturn::SUCCESS;
  }

  MyActuatorRmdHardwareInterface::CallbackReturn
  MyActuatorRmdHardwareInterface::on_deactivate(rclcpp_lifecycle::State const& previous_state) {
    return CallbackReturn::SUCCESS;
  }
  
  MyActuatorRmdHardwareInterface::CallbackReturn
  MyActuatorRmdHardwareInterface::on_error(rclcpp_lifecycle::State const& previous_state) {
    return CallbackReturn::SUCCESS;
  }

  MyActuatorRmdHardwareInterface::CallbackReturn
  MyActuatorRmdHardwareInterface::on_init(hardware_interface::HardwareInfo const& info) {
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> MyActuatorRmdHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interface {};
    return state_interface;
  }

  std::vector<hardware_interface::CommandInterface> MyActuatorRmdHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interface {};
    return command_interface;
  }

  hardware_interface::return_type prepare_command_mode_switch(std::vector<std::string> const& start_interfaces,
    std::vector<std::string> const& stop_interfaces) {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type perform_command_mode_switch(std::vector<std::string> const& start_interfaces,
    std::vector<std::string> const& stop_interfaces) {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type MyActuatorRmdHardwareInterface::read(rclcpp::Time const& time, rclcpp::Duration const& period) {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type MyActuatorRmdHardwareInterface::write(rclcpp::Time const& time, rclcpp::Duration const& period) {
    return hardware_interface::return_type::OK;
  }

}  // namespace myactuator_rmd_hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(myactuator_rmd_hardware::MyActuatorRmdHardwareInterface, hardware_interface::ActuatorInterface)
