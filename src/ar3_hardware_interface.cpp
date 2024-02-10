/**
 * @file ar3_hardware_interface.cpp
 * @author Zach Champion (zachchampion79@gmail.com)
 *
 * @version 1.0
 * @date 2023-09-25
 */

#include "ar3_hardware_interface/ar3_hardware_interface.hpp"

#include <chrono>

#include "ar3_hardware_interface/checksum.hpp"
#include "ar3_hardware_interface/serialize.h"

static constexpr uint32_t FW_VERSION = 6;

static constexpr size_t JOINT_COUNT = 6;
static constexpr double DEG_TO_RAD = M_PI / 180.0;
static constexpr double RAD_TO_DEG = 180.0 / M_PI;
static constexpr double ANGLE_TO_COBOT = RAD_TO_DEG * 1000.0;
static constexpr double ANGLE_FROM_COBOT = DEG_TO_RAD / 1000.0;

using namespace std;

namespace ar3_hardware_interface
{

ar3_hardware_interface::AR3HardwareInterface::AR3HardwareInterface() : messenger_(Messenger(1000))
{
}

hardware_interface::CallbackReturn
AR3HardwareInterface::on_init(const hardware_interface::HardwareInfo& info)
{
  // Verifies that hardware info is valid and can be parsed. Sets the variable `info_`, which
  // should be used in place of `info` for the rest of the function.
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Get serial port name and baud rate from hardware info
  prefix_ = info_.hardware_parameters["prefix"];
  serial_dev_name_ = info_.hardware_parameters["serial_port"];
  int raw_baud_rate = stoi(info_.hardware_parameters["baud_rate"]);
  switch (raw_baud_rate) {
    case 50:
      baud_rate_ = LibSerial::BaudRate::BAUD_50;
      break;
    case 75:
      baud_rate_ = LibSerial::BaudRate::BAUD_75;
      break;
    case 110:
      baud_rate_ = LibSerial::BaudRate::BAUD_110;
      break;
    case 134:
      baud_rate_ = LibSerial::BaudRate::BAUD_134;
      break;
    case 150:
      baud_rate_ = LibSerial::BaudRate::BAUD_150;
      break;
    case 200:
      baud_rate_ = LibSerial::BaudRate::BAUD_200;
      break;
    case 300:
      baud_rate_ = LibSerial::BaudRate::BAUD_300;
      break;
    case 600:
      baud_rate_ = LibSerial::BaudRate::BAUD_600;
      break;
    case 1200:
      baud_rate_ = LibSerial::BaudRate::BAUD_1200;
      break;
    case 1800:
      baud_rate_ = LibSerial::BaudRate::BAUD_1800;
      break;
    case 2400:
      baud_rate_ = LibSerial::BaudRate::BAUD_2400;
      break;
    case 4800:
      baud_rate_ = LibSerial::BaudRate::BAUD_4800;
      break;
    case 9600:
      baud_rate_ = LibSerial::BaudRate::BAUD_9600;
      break;
    case 19200:
      baud_rate_ = LibSerial::BaudRate::BAUD_19200;
      break;
    case 38400:
      baud_rate_ = LibSerial::BaudRate::BAUD_38400;
      break;
    case 57600:
      baud_rate_ = LibSerial::BaudRate::BAUD_57600;
      break;
    case 115200:
      baud_rate_ = LibSerial::BaudRate::BAUD_115200;
      break;
    case 230400:
      baud_rate_ = LibSerial::BaudRate::BAUD_230400;
      break;

    default:
      RCLCPP_ERROR(get_logger(), "Invalid baud rate: %d", raw_baud_rate);
      return CallbackReturn::ERROR;
  }

  // Initialize default command and state interfaces
  joints_.clear();
  joints_.reserve(JOINT_COUNT);
  for (size_t i = 0; i < JOINT_COUNT; ++i) {
    string name = prefix_ + "_joint_" + to_string(i);
    joints_.emplace_back(Joint{ .name_ = name });
  }
  gripper_name_ = prefix_ + "_gripper_servo";
  gripper_enabled_ = false;
  gripper_position_command_ = 0.0;
  gripper_position_state_ = 0.0;

  // Enable all joints that match the hardware info
  for (auto& info_joint : info_.joints) {
    // Enable gripper joint
    if (info_joint.name == gripper_name_) {
      if (gripper_enabled_) {
        RCLCPP_ERROR(get_logger(), "Duplicate gripper joint: %s", info_joint.name.c_str());
        return CallbackReturn::ERROR;
      } else {
        gripper_enabled_ = true;
      }
      continue;
    }

    // Enable other joints
    for (auto& joint : joints_) {
      if (joint.name_ == info_joint.name) {
        if (joint.enabled_) {
          RCLCPP_ERROR(get_logger(), "Duplicate joint: %s", info_joint.name.c_str());
          return CallbackReturn::ERROR;
        } else {
          joint.enabled_ = true;
        }
      }
    }
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
AR3HardwareInterface::on_shutdown(const rclcpp_lifecycle::State& previous_state)
{
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
AR3HardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  // Connect to serial port
  try {
    serial_port_.Open(serial_dev_name_);
    serial_port_.SetBaudRate(baud_rate_);
  } catch (const exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to open serial port: %s", e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_logger(), "Opened serial port");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
AR3HardwareInterface::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  // Close serial port
  try {
    serial_port_.Close();
  } catch (const exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to close serial port: %s", e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_logger(), "Closed serial port");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
AR3HardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  static uint8_t fw_version[] = {
    FW_VERSION & 0xFF,
    (FW_VERSION >> 8) & 0xFF,
    (FW_VERSION >> 16) & 0xFF,
    (FW_VERSION >> 24) & 0xFF,
  };
  static uint8_t all_joints[] = { 0b111111 };

  // Send init command with firmware version.
  try {
    auto logger = get_logger();
    uint32_t msg_id = messenger_.send_request(RequestType::Init, fw_version, sizeof(fw_version),
                                              serial_port_, logger);
    messenger_.wait_for_ack(msg_id, serial_port_, logger);
    RCLCPP_INFO(logger, "COBOT initialized");

    // // Calibrate the robot.
    // msg_id = messenger_.send_request(RequestType::Calibrate, all_joints, sizeof(all_joints),
    //                                  serial_port_, logger);
    // messenger_.wait_for_ack(msg_id, serial_port_, logger);
    // messenger_.wait_for_done(msg_id, serial_port_, logger);
    // RCLCPP_INFO(logger, "COBOT calibrated");

    // // Home the robot.
    // msg_id = messenger_.send_request(RequestType::GoHome, all_joints, sizeof(all_joints),
    //                                  serial_port_, logger);
    // messenger_.wait_for_ack(msg_id, serial_port_, logger);
    // messenger_.wait_for_done(msg_id, serial_port_, logger);
    // RCLCPP_INFO(logger, "COBOT homed");

  } catch (const exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to activate robot: %s", e.what());
    return CallbackReturn::ERROR;
  } catch (const CobotError& e) {
    RCLCPP_ERROR(get_logger(), "Failed to activate robot: %s", e.to_string().c_str());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
AR3HardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  // Send reset command.
  try {
    auto logger = get_logger();
    uint32_t msg_id = messenger_.send_request(RequestType::Reset, nullptr, 0, serial_port_, logger);
    messenger_.wait_for_ack(msg_id, serial_port_, logger);
    messenger_.wait_for_done(msg_id, serial_port_, logger);
  } catch (const exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to deactivate robot: %s", e.what());
    return CallbackReturn::ERROR;
  } catch (const CobotError& e) {
    RCLCPP_ERROR(get_logger(), "Failed to deactivate robot: %s", e.to_string().c_str());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

vector<hardware_interface::StateInterface> AR3HardwareInterface::export_state_interfaces()
{
  vector<hardware_interface::StateInterface> state_interfaces;
  for (auto& joint : joints_) {
    if (joint.enabled_) {
      state_interfaces.emplace_back(joint.name_, "position", &joint.position_state_);
      state_interfaces.emplace_back(joint.name_, "velocity", &joint.velocity_state_);
    }
  }
  if (gripper_enabled_) {
    state_interfaces.emplace_back(gripper_name_, "position", &gripper_position_state_);
  }
  return state_interfaces;
}

vector<hardware_interface::CommandInterface> AR3HardwareInterface::export_command_interfaces()
{
  vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto& joint : joints_) {
    if (joint.enabled_) {
      command_interfaces.emplace_back(joint.name_, "position", &joint.position_command_);
      command_interfaces.emplace_back(joint.name_, "velocity", &joint.velocity_command_);
    }
  }
  if (gripper_enabled_) {
    command_interfaces.emplace_back(gripper_name_, "position", &gripper_position_command_);
  }
  return command_interfaces;
}

rclcpp::Logger AR3HardwareInterface::get_logger() const
{
  return rclcpp::get_logger("ar3_hardware_interface");
}

hardware_interface::return_type AR3HardwareInterface::read(const rclcpp::Time& time,
                                                           const rclcpp::Duration& period)
{
  // Send request for joint positions.
  auto logger = get_logger();
  auto msg_id = messenger_.send_request(RequestType::GetJoints, nullptr, 0, serial_port_, logger);

  // Wait for valid response.
  unique_ptr<Response> response;
  while (true) {
    response = messenger_.wait_for_response(msg_id, serial_port_, logger);
    if (response->type == Response::Type::Joints) break;

    RCLCPP_WARN(logger, "Received unexpected response type: %d",
                static_cast<uint8_t>(response->type));
  }

  // Make sure there is a payload and it is the correct size.
  if (!(response->data)) {
    RCLCPP_ERROR(logger, "Received joints response with no payload");
    return hardware_interface::return_type::ERROR;
  }
  auto payload = response->data.value();
  auto payload_data = payload.data();
  auto payload_size = payload.size();
  if (payload_size < 1) {
    RCLCPP_ERROR(logger, "Received joints response with invalid payload size: %ld", payload.size());
    return hardware_interface::return_type::ERROR;
  }
  uint8_t received_joint_count = payload[0];
  size_t expected_size = 1 + received_joint_count * 8 + 1;
  if (payload_size != expected_size) {
    RCLCPP_ERROR(logger, "Received joints response with invalid payload size: %ld, expected %ld",
                 payload.size(), expected_size);
    return hardware_interface::return_type::ERROR;
  }

  // Update joint positions.
  for (int i = 0; i < JOINT_COUNT; ++i) {
    int32_t angle;
    int32_t speed;
    deserialize_int32(&angle, payload_data + 1 + i * 8);
    deserialize_int32(&speed, payload_data + 5 + i * 8);
    joints_[i].position_state_ = static_cast<double>(angle) * ANGLE_FROM_COBOT;
    joints_[i].velocity_state_ = static_cast<double>(speed) * ANGLE_FROM_COBOT;
  }

  // Update gripper servo position.
  uint8_t gripper_pos_deg = payload_data[expected_size - 1];
  gripper_position_state_ = static_cast<double>(gripper_pos_deg) * DEG_TO_RAD;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AR3HardwareInterface::write(const rclcpp::Time& time,
                                                            const rclcpp::Duration& period)
{
  // Create payload buffer.
  uint8_t payload_size =
      JOINT_COUNT * 8 + 1;  // Each joint has 8 bytes of data, plus 1 byte for gripper servo.
  vector<uint8_t> payload(payload_size);
  uint8_t* payload_data = payload.data();

  auto logger = get_logger();

  // Serialize joint commands.
  for (size_t i = 0; i < JOINT_COUNT; ++i) {
    // Convert to units expected by the robot.
    int32_t command_position = static_cast<int32_t>(joints_[i].position_command_ * ANGLE_TO_COBOT);
    int32_t command_speed = static_cast<int32_t>(joints_[i].velocity_command_ * ANGLE_TO_COBOT);

    // Serialize joint command.
    size_t command_position_offset = i * 8 + 0;
    size_t command_speed_offset = i * 8 + 4;
    serialize_int32(payload_data + command_position_offset, payload_size - command_position_offset,
                    command_position);
    serialize_int32(payload_data + command_speed_offset, payload_size - command_speed_offset,
                    command_speed);
  }

  // Serialize gripper servo command.
  uint8_t gripper_command_pos_deg = static_cast<uint8_t>(gripper_position_command_ * RAD_TO_DEG);
  payload[payload_size - 1] = gripper_command_pos_deg;

  // Send command.
  uint32_t msg_id = messenger_.send_request(RequestType::FollowTrajectory, payload_data,
                                            payload_size, serial_port_, logger);
  messenger_.wait_for_ack(msg_id, serial_port_, logger);

  return hardware_interface::return_type::OK;
}

}  // namespace ar3_hardware_interface

// This is required for pluginlib to find AR3HardwareInterface
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ar3_hardware_interface::AR3HardwareInterface,
                       hardware_interface::SystemInterface)