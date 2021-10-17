#ifndef USUSAMA_HARDWARE__DYNAMIXEL_HARDWARE_HPP_
#define USUSAMA_HARDWARE__DYNAMIXEL_HARDWARE_HPP_

#include <mecanum_io/mecanum_serial.hpp>

#include <hardware_interface/base_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_status_values.hpp>
#include <map>
#include <vector>

#include "ususama_hardware/visiblity_control.h"
#include "rclcpp/macros.hpp"

using hardware_interface::return_type;

namespace ususama_hardware
{
struct JointValue
{
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
};

struct Joint
{
  JointValue state{};
  JointValue command{};
};

enum class ControlMode {
  Position,
  Velocity,
  Torque,
  Currrent,
  ExtendedPosition,
  MultiTurn,
  CurrentBasedPosition,
  PWM,
};

class UsusamaHardware
: public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(UsusamaHardware)

  USUSAMA_HARDWARE_PUBLIC
  return_type configure(const hardware_interface::HardwareInfo & info) override;

  USUSAMA_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  USUSAMA_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  USUSAMA_HARDWARE_PUBLIC
  return_type start() override;

  USUSAMA_HARDWARE_PUBLIC
  return_type stop() override;

  USUSAMA_HARDWARE_PUBLIC
  return_type read() override;

  USUSAMA_HARDWARE_PUBLIC
  return_type write() override;

private:
  return_type enable_torque(const bool enabled);

  return_type set_control_mode(const ControlMode & mode, const bool force_set = false);

  return_type reset_command();

  //DynamixelWorkbench dynamixel_workbench_;
  //MecanumSerial mecanum_;
  std::unique_ptr<MecanumSerial> mecanum_;
  //std::map<const char * const, const ControlItem *> control_items_;
  std::vector<Joint> joints_;
  std::vector<uint8_t> joint_ids_;
  bool torque_enabled_{false};
  ControlMode control_mode_{ControlMode::Position};
  bool use_dummy_{false};
};
}  // namespace ususama_hardware

#endif  // USUSAMA_HARDWARE__DYNAMIXEL_HARDWARE_HPP_