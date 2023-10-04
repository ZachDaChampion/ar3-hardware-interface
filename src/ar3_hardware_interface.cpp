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

static constexpr uint32_t FW_VERSION = 3;

using namespace std;

namespace ar3_hardware_interface
{

ar3_hardware_interface::AR3HardwareInterface::AR3HardwareInterface() {}

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

  // Initialize joint command and state vectors
  auto joint_count = info_.joints.size();
  joint_position_command_.assign(joint_count, 0.0);
  joint_velocity_command_.assign(joint_count, 0.0);
  joint_position_.assign(joint_count, 0.0);
  joint_velocity_.assign(joint_count, 0.0);

  // Register joint interfaces
  for (const auto& joint : info_.joints) {
    joint_names_.push_back(joint.name);
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

    // Calibrate the robot.
    msg_id = messenger_.send_request(RequestType::Calibrate, all_joints, sizeof(all_joints),
                                     serial_port_, logger);
    messenger_.wait_for_ack(msg_id, serial_port_, logger);
    messenger_.wait_for_done(msg_id, serial_port_, logger);

    // Home the robot.
    msg_id = messenger_.send_request(RequestType::GoHome, all_joints, sizeof(all_joints),
                                     serial_port_, logger);
    messenger_.wait_for_ack(msg_id, serial_port_, logger);
    messenger_.wait_for_done(msg_id, serial_port_, logger);
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
  for (int i = 0; i < joint_names_.size(); ++i) {
    auto joint_name = joint_names_[i];
    state_interfaces.emplace_back(joint_name, "position", &joint_position_[i]);
    state_interfaces.emplace_back(joint_name, "velocity", &joint_velocity_[i]);
  }
  return state_interfaces;
}

vector<hardware_interface::CommandInterface> AR3HardwareInterface::export_command_interfaces()
{
  vector<hardware_interface::CommandInterface> command_interfaces;
  for (int i = 0; i < joint_names_.size(); ++i) {
    auto joint_name = joint_names_[i];
    command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[i]);
    command_interfaces.emplace_back(joint_name, "velocity", &joint_velocity_command_[i]);
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
    if (response->type != Response::Type::Joints) {
      RCLCPP_WARN(logger, "Received unexpected response type: %d", response->type);
      continue;
    }
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
    RCLCPP_ERROR(logger, "Received joints response with invalid payload size: %d", payload.size());
    return hardware_interface::return_type::ERROR;
  }
  uint8_t received_joint_count = payload[0];
  auto expected_size = 1 + received_joint_count * 8;
  if (payload_size != expected_size) {
    RCLCPP_ERROR(logger, "Received joints response with invalid payload size: %d, expected %d",
                 payload.size(), expected_size);
    return hardware_interface::return_type::ERROR;
  }

  // Figure out how many joints we can update.
  auto joint_count = min(received_joint_count, static_cast<uint8_t>(joint_names_.size()));

  // Update joint positions.
  for (int i = 0; i < joint_count; ++i) {
    int32_t angle;
    int32_t speed;
    deserialize_int32(&angle, payload_data + 1 + i * 8);
    deserialize_int32(&speed, payload_data + 5 + i * 8);
    joint_position_[i] = static_cast<double>(angle) / 1000.0;
    joint_velocity_[i] = static_cast<double>(speed) / 1000.0;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AR3HardwareInterface::write(const rclcpp::Time& time,
                                                            const rclcpp::Duration& period)
{
  // Create payload buffer.
  auto joint_count = joint_names_.size();
  uint8_t payload_size = joint_count * 9;  // Each joint has 9 bytes of data.
  vector<uint8_t> payload(payload_size);
  uint8_t* payload_data = payload.data();

  // Serialize joint commands.
  for (int i = 0; i < joint_count; ++i) {
    // Convert to units expected by the robot.
    uint8_t joint_id = static_cast<uint8_t>(i);
    int32_t command_position = static_cast<int32_t>(joint_position_command_[i] * 1000.0);
    int32_t command_speed = static_cast<int32_t>(joint_velocity_command_[i] * 1000.0);

    // Serialize joint command.
    size_t joint_id_offset = i * 9;
    size_t command_position_offset = joint_id_offset + 1;
    size_t command_speed_offset = command_position_offset + 4;
    serialize_uint8(payload_data + joint_id_offset, payload_size - joint_id_offset, joint_id);
    serialize_int32(payload_data + command_position_offset, payload_size - command_position_offset,
                    command_position);
    serialize_int32(payload_data + command_speed_offset, payload_size - command_speed_offset,
                    command_speed);
  }

  // Send command.
  auto logger = get_logger();
  uint32_t msg_id = messenger_.send_request(RequestType::MoveTo, payload.data(), payload_size,
                                            serial_port_, logger);
  messenger_.wait_for_ack(msg_id, serial_port_, logger);

  return hardware_interface::return_type::OK;
}

}  // namespace ar3_hardware_interface