#include "myactuator_rmd_state_broadcaster/myactuator_rmd_state_broadcaster.hpp"

#include <stddef.h>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcpputils/split.hpp"
#include "rcutils/logging_macros.h"
#include "std_msgs/msg/header.hpp"

namespace rclcpp_lifecycle
{
class State;
}  // namespace rclcpp_lifecycle

namespace myactuator_rmd_state_broadcaster
{
MyActuatorRmdStateBroadcaster::MyActuatorRmdStateBroadcaster() {
}

controller_interface::CallbackReturn MyActuatorRmdStateBroadcaster::on_init()
{
  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MyActuatorRmdStateBroadcaster::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration MyActuatorRmdStateBroadcaster::state_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;

  if (use_all_available_interfaces())
  {
    state_interfaces_config.type = controller_interface::interface_configuration_type::ALL;
  }
  else
  {
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & motor : params_.motors)
    {
      for (const auto & interface : params_.interfaces)
      {
        state_interfaces_config.names.push_back(motor + "/" + interface);
      }
    }
  }

  return state_interfaces_config;
}

controller_interface::CallbackReturn MyActuatorRmdStateBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!param_listener_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }
  params_ = param_listener_->get_params();

  if (use_all_available_interfaces())
  {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "'motors' or 'interfaces' parameter is empty. "
      "All available state interfaces will be published");
    params_.motors.clear();
    params_.interfaces.clear();
  }
  else
  {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Publishing state interfaces defined in 'motors' and 'interfaces' parameters.");
  }

  try
  {
    const std::string topic_name_prefix = params_.use_local_topics ? "~/" : "";

    motor_state_publisher_ = get_node()->create_publisher<myactuator_rmd_interfaces::msg::MotorStatus>(
      topic_name_prefix + "myactuator_rmd_states", rclcpp::SystemDefaultsQoS());

    realtime_motor_state_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<myactuator_rmd_interfaces::msg::MotorStatus>>(
        motor_state_publisher_);
  }
  catch (const std::exception & e)
  {
    // get_node() may throw, logging raw here
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MyActuatorRmdStateBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!init_motor_data())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "None of requested interfaces exist. Controller will not run.");
    return CallbackReturn::ERROR;
  }

  init_motor_state_msg();

  if (
    !use_all_available_interfaces() &&
    state_interfaces_.size() != (params_.motors.size() * params_.interfaces.size()))
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Not all requested interfaces exists. "
      "Check ControllerManager output for more detailed information.");
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MyActuatorRmdStateBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  motor_names_.clear();

  return CallbackReturn::SUCCESS;
}

template <typename T>
bool has_any_key(
  const std::unordered_map<std::string, T> & map, const std::vector<std::string> & keys)
{
  bool found_key = false;
  for (const auto & key_item : map)
  {
    const auto & key = key_item.first;
    if (std::find(keys.cbegin(), keys.cend(), key) != keys.cend())
    {
      found_key = true;
      break;
    }
  }
  return found_key;
}

bool MyActuatorRmdStateBroadcaster::init_motor_data()
{
  motor_names_.clear();
  if (state_interfaces_.empty())
  {
    return false;
  }

  // loop in reverse order, this maintains the order of values at retrieval time
  for (auto si = state_interfaces_.crbegin(); si != state_interfaces_.crend(); si++)
  {
    // initialize map if name is new
    if (name_if_value_mapping_.count(si->get_prefix_name()) == 0)
    {
      name_if_value_mapping_[si->get_prefix_name()] = {};
    }
    // add interface name
    std::string interface_name = si->get_interface_name();
    name_if_value_mapping_[si->get_prefix_name()][interface_name] = std::numeric_limits<double>::quiet_NaN();
  }

  // filter state interfaces that have at least one of the motor_states fields,
  // the rest will be ignored for this message
  for (const auto & name_ifv : name_if_value_mapping_)
  {
    const auto & interfaces_and_values = name_ifv.second;
    if (has_any_key(interfaces_and_values, {"rmd_error_code", 
                                            "rmd_temperature", 
                                            "rmd_brake", 
                                            "rmd_voltage",
                                            "rmd_current",
                                            "rmd_phase_a_current",
                                            "rmd_phase_b_current",
                                            "rmd_phase_c_current"}))
    {
      motor_names_.push_back(name_ifv.first);
    }
  }

  // Add extra motors from parameters, each motor will be added to motor_names_ and
  // name_if_value_mapping_ if it is not already there
  rclcpp::Parameter extra_motors;
  if (get_node()->get_parameter("extra_motors", extra_motors))
  {
    const std::vector<std::string> & extra_motors_names = extra_motors.as_string_array();
    for (const auto & extra_motor_name : extra_motors_names)
    {
      if (name_if_value_mapping_.count(extra_motor_name) == 0)
      {
        name_if_value_mapping_[extra_motor_name] = {
                                                    {"rmd_error_code", 0.0}, 
                                                    {"rmd_temperature", 0.0}, 
                                                    {"rmd_brake", 0.0},
                                                    {"rmd_voltage", 0.0},
                                                    {"rmd_current", 0.0},
                                                    {"rmd_phase_a_current", 0.0},
                                                    {"rmd_phase_b_current", 0.0},
                                                    {"rmd_phase_c_current", 0.0}};
        motor_names_.push_back(extra_motor_name);
      }
    }
  }
  return true;
}

void MyActuatorRmdStateBroadcaster::init_motor_state_msg()
{
  const size_t num_motors = motor_names_.size();

  // default initialization for motor state message
  auto & motor_state_msg = realtime_motor_state_publisher_->msg_;
  motor_state_msg.name = motor_names_;
  motor_state_msg.voltage.resize(num_motors, std::numeric_limits<double>::quiet_NaN());
  motor_state_msg.current.resize(num_motors, std::numeric_limits<double>::quiet_NaN());
  motor_state_msg.temperature.resize(num_motors, std::numeric_limits<std::int64_t>::quiet_NaN());
  motor_state_msg.error_code.resize(num_motors, std::numeric_limits<std::int64_t>::quiet_NaN());
  motor_state_msg.brake.resize(num_motors, false);
  motor_state_msg.current_phases.resize(num_motors);
  for (size_t i = 0; i < num_motors; ++i) {
    motor_state_msg.current_phases[i].name.resize(3);
    motor_state_msg.current_phases[i].name[0] =  "a";
    motor_state_msg.current_phases[i].name[1] =  "b";
    motor_state_msg.current_phases[i].name[2] =  "c";
    motor_state_msg.current_phases[i].current.resize(3); 
    motor_state_msg.current_phases[i].current[0] = std::numeric_limits<double>::quiet_NaN();
    motor_state_msg.current_phases[i].current[1] = std::numeric_limits<double>::quiet_NaN();
    motor_state_msg.current_phases[i].current[2] = std::numeric_limits<double>::quiet_NaN();
  }
}

bool MyActuatorRmdStateBroadcaster::use_all_available_interfaces() const
{
  return params_.motors.empty() || params_.interfaces.empty();
}

double get_value(
  const std::unordered_map<std::string, std::unordered_map<std::string, double>> & map,
  const std::string & name, const std::string & interface_name)
{
  const auto & interfaces_and_values = map.at(name);
  const auto interface_and_value = interfaces_and_values.find(interface_name);
  if (interface_and_value != interfaces_and_values.cend())
  {
    return interface_and_value->second;
  }
  else
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
}

controller_interface::return_type MyActuatorRmdStateBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  for (const auto & state_interface : state_interfaces_)
  {
    std::string interface_name = state_interface.get_interface_name();
    name_if_value_mapping_[state_interface.get_prefix_name()][interface_name] =
      state_interface.get_value();
    RCLCPP_DEBUG(
      get_node()->get_logger(), "%s: %f\n", state_interface.get_name().c_str(),
      state_interface.get_value());
  }

  if (realtime_motor_state_publisher_ && realtime_motor_state_publisher_->trylock())
  {
    auto & motor_state_msg = realtime_motor_state_publisher_->msg_;

    motor_state_msg.header.stamp = time;

    // update motor state message 
    for (size_t i = 0; i < motor_names_.size(); ++i)
    {
      motor_state_msg.voltage[i] = get_value(name_if_value_mapping_, motor_names_[i], "rmd_voltage");
      motor_state_msg.current[i] = get_value(name_if_value_mapping_, motor_names_[i], "rmd_current");
      motor_state_msg.temperature[i] = static_cast<long int>(get_value(name_if_value_mapping_, motor_names_[i], "rmd_temperature"));
      motor_state_msg.error_code[i] = static_cast<long int>(get_value(name_if_value_mapping_, motor_names_[i], "rmd_error_code"));
      motor_state_msg.brake[i] = static_cast<bool>(get_value(name_if_value_mapping_, motor_names_[i], "rmd_brake"));
      motor_state_msg.current_phases[i].current[0] = get_value(name_if_value_mapping_, motor_names_[i], "rmd_current_phase_a");
      motor_state_msg.current_phases[i].current[1] = get_value(name_if_value_mapping_, motor_names_[i], "rmd_current_phase_b");
      motor_state_msg.current_phases[i].current[2] = get_value(name_if_value_mapping_, motor_names_[i], "rmd_current_phase_c");
    }
    realtime_motor_state_publisher_->unlockAndPublish();
  }


  return controller_interface::return_type::OK;
}

}  // namespace myactuator_rmd_state_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  myactuator_rmd_state_broadcaster::MyActuatorRmdStateBroadcaster, controller_interface::ControllerInterface)