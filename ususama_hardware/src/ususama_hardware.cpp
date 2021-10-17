#include "ususama_hardware/ususama_hardware.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ususama_hardware
{
constexpr const char * kUsusamaHardware = "UsusamaHardware";
constexpr uint8_t kGoalPositionIndex = 0;
constexpr uint8_t kGoalVelocityIndex = 1;
constexpr uint8_t kPresentPositionVelocityCurrentIndex = 0;
constexpr const char * kGoalPositionItem = "Goal_Position";
constexpr const char * kGoalVelocityItem = "Goal_Velocity";
constexpr const char * kMovingSpeedItem = "Moving_Speed";
constexpr const char * kPresentPositionItem = "Present_Position";
constexpr const char * kPresentVelocityItem = "Present_Velocity";
constexpr const char * kPresentSpeedItem = "Present_Speed";
constexpr const char * kPresentCurrentItem = "Present_Current";
constexpr const char * kPresentLoadItem = "Present_Load";

return_type UsusamaHardware::configure(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kUsusamaHardware), "configure");
  if (configure_default(info) != return_type::OK) {
    return return_type::ERROR;
  }

  joints_.resize(info_.joints.size(), Joint());
  joint_ids_.resize(info_.joints.size(), 0);

  for (uint i = 0; i < info_.joints.size(); i++) {
    joint_ids_[i] = std::stoi(info_.joints[i].parameters.at("id"));
    joints_[i].state.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.effort = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.effort = std::numeric_limits<double>::quiet_NaN();
    RCLCPP_INFO(rclcpp::get_logger(kUsusamaHardware), "joint_id %d: %d", i, joint_ids_[i]);
  }

  if (
    info_.hardware_parameters.find("use_dummy") != info_.hardware_parameters.end() &&
    info_.hardware_parameters.at("use_dummy") == "true") {
    use_dummy_ = true;
    RCLCPP_INFO(rclcpp::get_logger(kUsusamaHardware), "dummy mode");
    status_ = hardware_interface::status::CONFIGURED;
    return return_type::OK;
  }

  auto usb_port = info_.hardware_parameters.at("usb_port");
  auto baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));
  const char * log = nullptr;

  RCLCPP_INFO(rclcpp::get_logger(kUsusamaHardware), "usb_port: %s", usb_port.c_str());
  RCLCPP_INFO(rclcpp::get_logger(kUsusamaHardware), "baud_rate: %d", baud_rate);

  //mecanum_ = MecanumSerial("/dev/ttyACM0", 115200);
  mecanum_ = std::make_unique<MecanumSerial>("/dev/ttyACM0", 115200);

  status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface> UsusamaHardware::export_state_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kUsusamaHardware), "export_state_interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].state.position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].state.velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joints_[i].state.effort));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> UsusamaHardware::export_command_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kUsusamaHardware), "export_command_interfaces");
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].command.position));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].command.velocity));
  }
  return command_interfaces;
}

return_type UsusamaHardware::start()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kUsusamaHardware), "start");
  for (uint i = 0; i < joints_.size(); i++) {
    if (use_dummy_ && std::isnan(joints_[i].state.position)) {
      joints_[i].state.position = 0.0;
      joints_[i].state.velocity = 0.0;
      joints_[i].state.effort = 0.0;
    }
  }
  read();
  reset_command();
  write();

  status_ = hardware_interface::status::STARTED;
  return return_type::OK;
}

return_type UsusamaHardware::stop()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kUsusamaHardware), "stop");
  status_ = hardware_interface::status::STOPPED;
  return return_type::OK;
}

return_type UsusamaHardware::read()
{
  if (use_dummy_) {
    return return_type::OK;
  }

  std::vector<uint8_t> ids(info_.joints.size(), 0);
  std::vector<int32_t> positions(info_.joints.size(), 0);
  std::vector<int32_t> velocities(info_.joints.size(), 0);
  std::vector<int32_t> currents(info_.joints.size(), 0);

  std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());
  const char * log = nullptr;
  
  mecanum_ -> update();
  
  for (uint i = 0; i < ids.size(); i++) {
    //joints_[i].state.position = mecanum_.PositionX();//dynamixel_workbench_.convertValue2Radian(ids[i], positions[i]);
    //joints_[i].state.velocity = mecanum_.VelocityX();//dynamixel_workbench_.convertValue2Velocity(ids[i], velocities[i]);
    //joints_[i].state.effort = 0;//dynamixel_workbench_.convertValue2Current(currents[i]);
    if(i == 0){
      joints_[0].state.position = mecanum_ -> PositionX();
      joints_[0].state.velocity = mecanum_ -> VelocityX();
      joints_[0].state.effort = 0;
    }
    if(i == 1){
      joints_[1].state.position = mecanum_ -> PositionY();
      joints_[1].state.velocity = mecanum_ -> VelocityY();
      joints_[1].state.effort = 0;
    }
    if(i == 2){
      joints_[2].state.position = mecanum_ -> PositionTheta();
      joints_[2].state.velocity = mecanum_ -> VelocityTheta();
      joints_[2].state.effort = 0;
    }
  }
  //std::cout << mecanum_ -> PositionTheta() << std::endl;
  //double theta = mecanum_ -> PositionTheta();
  return return_type::OK;
}

return_type UsusamaHardware::write()
{
  if (use_dummy_) {
    for (auto & joint : joints_) {
      joint.state.position = joint.command.position;
    }

    return return_type::OK;
  }

  std::vector<uint8_t> ids(info_.joints.size(), 0);
  std::vector<int32_t> commands(info_.joints.size(), 0);

  std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());
  const char * log = nullptr;

  if (std::any_of(
        joints_.cbegin(), joints_.cend(), [](auto j) { return j.command.velocity != 0.0; })) {
    // Velocity control
    set_control_mode(ControlMode::Velocity);
    /*for (uint i = 0; i < ids.size(); i++) {
      commands[i] = dynamixel_workbench_.convertVelocity2Value(
        ids[i], static_cast<float>(joints_[i].command.velocity));
    }
    if (!dynamixel_workbench_.syncWrite(
          kGoalVelocityIndex, ids.data(), ids.size(), commands.data(), 1, &log)) {
      RCLCPP_FATAL(rclcpp::get_logger(kUsusamaHardware), "%s", log);
    }*/
    int vel_x = 0;
    int vel_y = 0;
    int vel_theta = 0;
    for (uint i = 0; i < ids.size(); i++) {
      if(i == 0)  vel_x = joints_[i].command.velocity;
      if(i == 1)  vel_y = joints_[i].command.velocity;
      if(i == 2)  vel_theta = joints_[i].command.velocity;
    }
    mecanum_ -> setVelocity( vel_x, vel_y, vel_theta);
    return return_type::OK;
  } else if (std::any_of(
               joints_.cbegin(), joints_.cend(), [](auto j) { return j.command.effort != 0.0; })) {
    // Effort control
    RCLCPP_FATAL(rclcpp::get_logger(kUsusamaHardware), "Effort control is not implemented");
    return return_type::ERROR;
  }

  // Position control
  /*set_control_mode(ControlMode::Position);
  for (uint i = 0; i < ids.size(); i++){
    mecanum_.setVelocity( joints_[i].command.velocity, 0, 0);
  }*/
  /*for (uint i = 0; i < ids.size(); i++) {
    commands[i] = dynamixel_workbench_.convertRadian2Value(
      ids[i], static_cast<float>(joints_[i].command.position));
  }
  if (!dynamixel_workbench_.syncWrite(
        kGoalPositionIndex, ids.data(), ids.size(), commands.data(), 1, &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kUsusamaHardware), "%s", log);
  }*/

  return return_type::OK;
}

return_type UsusamaHardware::enable_torque(const bool enabled)
{
  /*const char * log = nullptr;

  if (enabled && !torque_enabled_) {
    for (uint i = 0; i < info_.joints.size(); ++i) {
      if (!dynamixel_workbench_.torqueOn(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kUsusamaHardware), "%s", log);
        return return_type::ERROR;
      }
    }
    reset_command();
    RCLCPP_INFO(rclcpp::get_logger(kUsusamaHardware), "Torque enabled");
  } else if (!enabled && torque_enabled_) {
    for (uint i = 0; i < info_.joints.size(); ++i) {
      if (!dynamixel_workbench_.torqueOff(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kUsusamaHardware), "%s", log);
        return return_type::ERROR;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger(kUsusamaHardware), "Torque disabled");
  }*/

  torque_enabled_ = enabled;
  return return_type::OK;
}

return_type UsusamaHardware::set_control_mode(const ControlMode & mode, const bool force_set)
{
  const char * log = nullptr;

  if (mode == ControlMode::Velocity && (force_set || control_mode_ != ControlMode::Velocity)) {
    bool torque_enabled = torque_enabled_;
    if (torque_enabled) {
      enable_torque(false);
    }

    /*for (uint i = 0; i < joint_ids_.size(); ++i) {
      if (!dynamixel_workbench_.setVelocityControlMode(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kUsusamaHardware), "%s", log);
        return return_type::ERROR;
      }
    }*/
    RCLCPP_INFO(rclcpp::get_logger(kUsusamaHardware), "Velocity control");
    control_mode_ = ControlMode::Velocity;

    if (torque_enabled) {
      enable_torque(true);
    }
  } else if (
    mode == ControlMode::Position && (force_set || control_mode_ != ControlMode::Position)) {
    bool torque_enabled = torque_enabled_;
    if (torque_enabled) {
      enable_torque(false);
    }

    for (uint i = 0; i < joint_ids_.size(); ++i) {
      /*if (!dynamixel_workbench_.setPositionControlMode(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kUsusamaHardware), "%s", log);
        return return_type::ERROR;
      }*/
    }
    RCLCPP_INFO(rclcpp::get_logger(kUsusamaHardware), "Position control");
    control_mode_ = ControlMode::Position;

    if (torque_enabled) {
      enable_torque(true);
    }
  } else if (control_mode_ != ControlMode::Velocity && control_mode_ != ControlMode::Position) {
    RCLCPP_FATAL(
      rclcpp::get_logger(kUsusamaHardware), "Only position/velocity control are implemented");
    return return_type::ERROR;
  }

  return return_type::OK;
}

return_type UsusamaHardware::reset_command()
{
  for (uint i = 0; i < joints_.size(); i++) {
    joints_[i].command.position = joints_[i].state.position;
    joints_[i].command.velocity = 0.0;
    joints_[i].command.effort = 0.0;
  }

  return return_type::OK;
}

}  // namespace ususama_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ususama_hardware::UsusamaHardware, hardware_interface::SystemInterface)