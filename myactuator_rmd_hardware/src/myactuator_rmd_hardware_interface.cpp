#include "myactuator_rmd_hardware/myactuator_rmd_hardware_interface.hpp"

#include <iostream>

#include <atomic>
#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <myactuator_rmd/actuator_interface.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>


#include "myactuator_rmd_hardware/conversions.hpp"


namespace myactuator_rmd_hardware {

  using CallbackReturn = MyActuatorRmdHardwareInterface::CallbackReturn;

  MyActuatorRmdHardwareInterface::~MyActuatorRmdHardwareInterface() {
    // If the controller manager is shutdown with Ctrl + C the on_deactivate methods won't be called!
    // We therefore shut down the actuator here.
    on_cleanup(rclcpp_lifecycle::State());
    return;
  }

  CallbackReturn MyActuatorRmdHardwareInterface::on_configure(rclcpp_lifecycle::State const& /*previous_state*/) {
    if (info_.hardware_parameters.find("ifname") != info_.hardware_parameters.end()) {
      ifname_ = info_.hardware_parameters["ifname"];
    } else {
      RCLCPP_FATAL(getLogger(), "Could not parse CAN interface name!");
      return CallbackReturn::ERROR;
    }
    if (info_.hardware_parameters.find("actuator_id") != info_.hardware_parameters.end()) {
      actuator_id_ = std::stoi(info_.hardware_parameters["actuator_id"]);
    } else {
      RCLCPP_FATAL(getLogger(), "Could not parse CAN actuator id!");
      return CallbackReturn::ERROR;
    }
    if (info_.hardware_parameters.find("torque_constant") != info_.hardware_parameters.end()) {
      torque_constant_ = std::stod(info_.hardware_parameters["torque_constant"]);
    } else {
      torque_constant_ = std::numeric_limits<double>::quiet_NaN();
      RCLCPP_ERROR(getLogger(), "Could not parse torque constant, won't be able to use torque interface!");
    }
    if (info_.hardware_parameters.find("max_velocity") != info_.hardware_parameters.end()) {
      max_velocity_ = std::stod(info_.hardware_parameters["max_velocity"]);
    } else {
      max_velocity_ = 720.0;
      RCLCPP_INFO(getLogger(), "Max velocity not set, defaulting to '%f'", max_velocity_);
    }
    cycle_time_ = std::chrono::milliseconds(1);

    driver_ = std::make_unique<myactuator_rmd::CanDriver>(ifname_);
    actuator_interface_ = std::make_unique<myactuator_rmd::ActuatorInterface>(*driver_, actuator_id_);
    if (!actuator_interface_) {
      RCLCPP_INFO(getLogger(), "Failed to create actuator interface!");
      return CallbackReturn::ERROR;
    }
    std::string const motor_model {actuator_interface_->getMotorModel()};
    RCLCPP_INFO(getLogger(), "Started actuator interface for actuator model '%s'!", motor_model.c_str());
    stop_command_thread_.store(false);
    if (!startCommandThread(cycle_time_)) {
      RCLCPP_FATAL(getLogger(), "Failed to start command thread!");
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }
      
  CallbackReturn MyActuatorRmdHardwareInterface::on_cleanup(rclcpp_lifecycle::State const& /*previous_state*/) {
    stopCommandThread();
    if (actuator_interface_) {
      actuator_interface_->shutdownMotor();
    }
    return CallbackReturn::SUCCESS;
  }
  
  CallbackReturn MyActuatorRmdHardwareInterface::on_shutdown(rclcpp_lifecycle::State const& /*previous_state*/) {
    stopCommandThread();
    if (actuator_interface_) {
      actuator_interface_->shutdownMotor();
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MyActuatorRmdHardwareInterface::on_activate(rclcpp_lifecycle::State const& /*previous_state*/) {
    RCLCPP_INFO(getLogger(), "Actuator successfully started!");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MyActuatorRmdHardwareInterface::on_deactivate(rclcpp_lifecycle::State const& /*previous_state*/) {
    stopCommandThread();
    if (actuator_interface_) {
      actuator_interface_->stopMotor();
    }
    RCLCPP_INFO(getLogger(), "Actuator successfully stopped!");
    return CallbackReturn::SUCCESS;
  }
  
  CallbackReturn MyActuatorRmdHardwareInterface::on_error(rclcpp_lifecycle::State const& /*previous_state*/) {
    stopCommandThread();
    if (actuator_interface_) {
      actuator_interface_->stopMotor();
      actuator_interface_->reset();
      RCLCPP_INFO(getLogger(), "Actuator reset!");
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MyActuatorRmdHardwareInterface::on_init(hardware_interface::HardwareInfo const& info) {
    if (hardware_interface::ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS) {
      return CallbackReturn::ERROR;
    }

    position_state_ = 0.0;
    velocity_state_ = 0.0;
    effort_state_ = 0.0;
    position_command_ = 0.0;
    velocity_command_ = 0.0;
    effort_command_ = 0.0;

    position_interface_running_.store(false);
    velocity_interface_running_.store(false);
    effort_interface_running_.store(false);

    if (info_.joints.size() != 1) {
      RCLCPP_FATAL(getLogger(), "Expected a single joint but got %zu joints.", info_.joints.size());
      return CallbackReturn::ERROR;
    }
    hardware_interface::ComponentInfo const& joint = info_.joints.at(0);

    if (joint.command_interfaces.size() != 3) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has %zu command interfaces found. 3 expected.",
        joint.name.c_str(), joint.command_interfaces.size()
      );
      return CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION &&
        joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY &&
        joint.command_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(getLogger(),
        "Joint '%s' has unexpected command interface '%s'. Expected '%s', '%s' and '%s' ",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
        hardware_interface::HW_IF_EFFORT
      );
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has %zu state interfaces. 3 expected.",
        joint.name.c_str(), joint.state_interfaces.size()
      );
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected state interface '%s'. Expected '%s'",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION
      );
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected state interface '%s'. Expected '%s'",
        joint.name.c_str(), joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY
      );
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected state interface '%s'. Expected '%s'",
        joint.name.c_str(), joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_EFFORT
      );
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> MyActuatorRmdHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces {};
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints.at(0).name, hardware_interface::HW_IF_POSITION, &position_state_)
    );
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints.at(0).name, hardware_interface::HW_IF_VELOCITY, &velocity_state_)
    );
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints.at(0).name, hardware_interface::HW_IF_EFFORT, &effort_state_)
    );
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> MyActuatorRmdHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces {};
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints.at(0).name, hardware_interface::HW_IF_POSITION, &position_command_)
    );
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints.at(0).name, hardware_interface::HW_IF_VELOCITY, &velocity_command_)
    );
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints.at(0).name, hardware_interface::HW_IF_EFFORT, &effort_command_)
    );
    return command_interfaces;
  }

  hardware_interface::return_type MyActuatorRmdHardwareInterface::prepare_command_mode_switch(
    std::vector<std::string> const& start_interfaces,
    std::vector<std::string> const& stop_interfaces) {
    if (start_interfaces.size() == 1 ) {
     auto const& start_interface {start_interfaces.at(0)};
      if (start_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_POSITION) {
        if (velocity_interface_running_ || effort_interface_running_) {
          RCLCPP_ERROR(getLogger(), "Failed to switch to position interface!");
          return hardware_interface::return_type::ERROR;
        } position_interface_running_.store(true);
        RCLCPP_INFO(getLogger(), "Switched to position interface!");
      } else if (start_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_VELOCITY) {
        if (position_interface_running_ || effort_interface_running_) {
          RCLCPP_ERROR(getLogger(), "Failed to switch to velocity interface!");
          return hardware_interface::return_type::ERROR;
        } velocity_interface_running_.store(true);
        RCLCPP_INFO(getLogger(), "Switched to velocity interface!");
      } else if (start_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_EFFORT) {
        if (position_interface_running_ || velocity_interface_running_) {
          RCLCPP_ERROR(getLogger(), "Failed to switch to effort interface!");
          return hardware_interface::return_type::ERROR;
        } if (!std::isnan(torque_constant_)) {
          effort_interface_running_.store(true);
          RCLCPP_INFO(getLogger(), "Switched to effort interface!");
        } else {
          RCLCPP_ERROR(getLogger(), "Failed to switch to effort interface!");
          return hardware_interface::return_type::ERROR;
        }
      }
    }else if (start_interfaces.size() > 1 ) {
      RCLCPP_ERROR(getLogger(), "Expected a single joint but got %zu interfaces to start.", start_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }

    if (stop_interfaces.size() == 1 ) {
      auto const& stop_interface {stop_interfaces.at(0)};
      if (stop_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_POSITION) {
        position_interface_running_.store(false);
        RCLCPP_INFO(getLogger(), "Stop position interface!");
      } else if (stop_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_VELOCITY) {
        velocity_interface_running_.store(false);
        RCLCPP_INFO(getLogger(), "Stop velocity interface!");
      } else if (stop_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_EFFORT) {
        effort_interface_running_.store(false);
        RCLCPP_INFO(getLogger(), "Stop effort interface!");
      }
    }else if (stop_interfaces.size() > 1 ) {
      RCLCPP_ERROR(getLogger(), "Expected a single joint but got %zu interfaces to stop.", stop_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type MyActuatorRmdHardwareInterface::read(rclcpp::Time const& /*time*/,
    rclcpp::Duration const& /*period*/) {
    position_state_ = degToRad(feedback_.load().shaft_angle);
    velocity_state_ = degToRad(feedback_.load().shaft_speed);
    effort_state_ = currentToTorque(feedback_.load().current, torque_constant_);
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type MyActuatorRmdHardwareInterface::write(rclcpp::Time const& /*time*/,
    rclcpp::Duration const& /*period*/) {
    // TODO: Make sure that all commands are finite
    if (position_interface_running_) {
      command_thread_position_.store(radToDeg(position_command_));
    } else if (velocity_interface_running_) {
      command_thread_velocity_.store(radToDeg(velocity_command_));
    } else if (effort_interface_running_) {
      command_thread_effort_.store(effort_command_);
    }
    return hardware_interface::return_type::OK;
  }

  rclcpp::Logger MyActuatorRmdHardwareInterface::getLogger() {
    return rclcpp::get_logger("MyActuatorRmdHardwareInterface");
  }

  void MyActuatorRmdHardwareInterface::commandThread(std::chrono::milliseconds const& cycle_time) {
    while (!stop_command_thread_) {
      auto const now {std::chrono::steady_clock::now()};
      auto const wakeup_time {now + cycle_time};
      if (position_interface_running_) {
        feedback_ = actuator_interface_->sendPositionAbsoluteSetpoint(command_thread_position_.load(), max_velocity_);
      } else if (velocity_interface_running_) {
        feedback_ = actuator_interface_->sendVelocitySetpoint(command_thread_velocity_.load());
      } else if (effort_interface_running_) {
        feedback_ = actuator_interface_->sendTorqueSetpoint(command_thread_effort_.load(), torque_constant_);
      } else{
        actuator_interface_->stopMotor();
        feedback_ = actuator_interface_->getMotorStatus2();
      }
      std::this_thread::sleep_until(wakeup_time);
    }
    return;
  }

  bool MyActuatorRmdHardwareInterface::startCommandThread(std::chrono::milliseconds const& cycle_time) {
    if (!command_thread_.joinable()) {
      command_thread_ = std::thread(&MyActuatorRmdHardwareInterface::commandThread, this, cycle_time);
    } else {
      RCLCPP_WARN(getLogger(), "Could not start command thread, command thread already running!");
      return false;
    }
    return true;
  }

  void MyActuatorRmdHardwareInterface::stopCommandThread() {
    if (command_thread_.joinable()) {
      stop_command_thread_.store(true);
      command_thread_.join();
    } else {
      RCLCPP_WARN(getLogger(), "Could not stop command thread: Not running!");
    }
    return;
  }

}  // namespace myactuator_rmd_hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(myactuator_rmd_hardware::MyActuatorRmdHardwareInterface, hardware_interface::ActuatorInterface)
