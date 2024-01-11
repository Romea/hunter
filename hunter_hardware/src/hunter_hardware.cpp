// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


// std
#include <limits>
#include <string>
#include <vector>

// ros2
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

// romea
#include "romea_core_common/math/Algorithm.hpp"
#include "romea_mobile_base_hardware/hardware_info.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/FowardOneAxleSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/InverseOneAxleSteeringKinematic.hpp"

// local
#include "hunter_hardware/hunter_hardware.hpp"

namespace
{
size_t FRONT_STEERING_MOTOR_ID_ = 0;
size_t REAR_LEFT_SPINNING_MOTOR_ID_ = 1;
size_t REAR_RIGHT_SPINNING_MOTOR_ID_ = 2;
}

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
HunterHardware::HunterHardware()
: HardwareSystemInterface<HardwareInterface1FAS2RWD>("HunterHardware"),
  robot_(ProtocolVersion::AGX_V2),
  // front_wheel_radius_(0),
  rear_wheel_radius_(0),
  // wheelbase_(0),
  // front_track_(0),
  front_steering_angle_measure_(0),
  rear_left_wheel_angular_speed_measure_(0),
  rear_right_wheel_angular_speed_measure_(0),
  front_steering_angle_command_(0),
  rear_left_wheel_angular_speed_command_(0),
  rear_right_wheel_angular_speed_command_(0)
{
}

//-----------------------------------------------------------------------------
HunterHardware::~HunterHardware()
{
  // force deactive when interface has not been deactivated by controller manager but by ctrl-c
  if (lifecycle_state_.id() == 3) {
    on_deactivate(lifecycle_state_);
  }
}


//-----------------------------------------------------------------------------
hardware_interface::return_type HunterHardware::connect_()
{
  // RCLCPP_ERROR(rclcpp::get_logger("HunterHardware"), "Init communication with robot");
  // To be implememented

  if (robot_.Connect("can0")) {
    robot_.EnableCommandedMode();
    send_null_command_();
    return hardware_interface::return_type::OK;
  } else {
    return hardware_interface::return_type::ERROR;
  }
}

//-----------------------------------------------------------------------------
hardware_interface::return_type HunterHardware::disconnect_()
{
  // RCLCPP_ERROR(rclcpp::get_logger("HunterHardware"), "Close communication with robot");
  // To be implememented

  send_null_command_();
  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
hardware_interface::return_type HunterHardware::load_info_(
  const hardware_interface::HardwareInfo & hardware_info)
{
  try {
    // Get some info from ros2_control item of robot urdf file
    // wheelbase_ = get_parameter<double>(hardware_info, "wheelbase");
    // front_track_ = get_parameter<double>(hardware_info, "front_track");
    // front_wheel_radius_ = get_parameter<float>(hardware_info, "front_wheel_radius");
    rear_wheel_radius_ = get_parameter<float>(hardware_info, "rear_wheel_radius");
    return hardware_interface::return_type::OK;
  } catch (std::runtime_error & e) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("HunterHardware"), e.what());
    return hardware_interface::return_type::ERROR;
  }
}

//-----------------------------------------------------------------------------
void HunterHardware::send_null_command_()
{
  robot_.SetMotionCommand(0, 0);
}


//-----------------------------------------------------------------------------
#if ROS_DISTRO == ROS_GALACTIC
hardware_interface::return_type HunterHardware::read()
#else
hardware_interface::return_type HunterHardware::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
#endif
{
  try {
    //TODO (JEAN) to be finished
    auto actuator = robot_.GetActuatorState();
    // rear_left_wheel_angular_speed_measure_ =
    //   actuator.actuator_hs_state[FRONT_STEERING_MOTOR_ID_].pulse_count * ? ;
    // rear_left_wheel_angular_speed_measure_ =
    //   actuator.actuator_hs_state[REAR_LEFT_SPINNING_MOTOR_ID_].rpm * ? / rear_wheel_radius_;
    // rear_right_wheel_angular_speed_measure_ =
    //   actuator.actuator_hs_state[REAR_LEFT_SPINNING_MOTOR_ID_].rpm * ? / rear_wheel_radius_;

    set_hardware_state_();
    return hardware_interface::return_type::OK;
  } catch (std::runtime_error & e) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("HunterHardware"), e.what());
    return hardware_interface::return_type::ERROR;
  }
}


//-----------------------------------------------------------------------------
#if ROS_DISTRO == ROS_GALACTIC
hardware_interface::return_type HunterHardware::write()
# else
hardware_interface::return_type HunterHardware::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
#endif
{
  // RCLCPP_ERROR(rclcpp::get_logger("HunterHardware"), "Send command to robot");
  get_hardware_command_();

  auto linear_velocity_command = 0.5 * rear_wheel_radius_ * (
    rear_left_wheel_angular_speed_command_ + rear_right_wheel_angular_speed_command_);

  robot_.SetMotionCommand(linear_velocity_command, front_steering_angle_command_);
  return hardware_interface::return_type::OK;
}


//-----------------------------------------------------------------------------
void HunterHardware::set_hardware_state_()
{
  core::HardwareState1FAS2RWD state;
  state.frontAxleSteeringAngle = front_steering_angle_measure_;
  state.rearLeftWheelSpinningMotion.velocity = rear_left_wheel_angular_speed_measure_;
  state.rearRightWheelSpinningMotion.velocity = rear_right_wheel_angular_speed_measure_;
  this->hardware_interface_->set_state(state);
}

//-----------------------------------------------------------------------------
void HunterHardware::get_hardware_command_()
{
  core::HardwareCommand1FAS2RWD command = hardware_interface_->get_command();

  front_steering_angle_command_ = command.frontAxleSteeringAngle;
  rear_left_wheel_angular_speed_command_ = command.rearLeftWheelSpinningSetPoint;
  rear_right_wheel_angular_speed_command_ = command.rearRightWheelSpinningSetPoint;
}

}  // namespace ros2
}  // namespace romea


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(romea::ros2::HunterHardware, hardware_interface::SystemInterface)
