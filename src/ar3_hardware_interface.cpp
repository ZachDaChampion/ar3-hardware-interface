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

static constexpr uint32_t FW_VERSION = 3;

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
  int raw_baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
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
  } catch (const std::exception& e) {
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
  } catch (const std::exception& e) {
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
  } catch (const std::exception& e) {
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
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to deactivate robot: %s", e.what());
    return CallbackReturn::ERROR;
  } catch (const CobotError& e) {
    RCLCPP_ERROR(get_logger(), "Failed to deactivate robot: %s", e.to_string().c_str());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type AR3HardwareInterface::read(const rclcpp::Time& time,
                                                           const rclcpp::Duration& period)
{
  // TODO: Implement read
  return hardware_interface::return_type();
}

hardware_interface::return_type AR3HardwareInterface::write(const rclcpp::Time& time,
                                                            const rclcpp::Duration& period)
{
  // TODO: Implement write
  return hardware_interface::return_type();
}

std::vector<hardware_interface::StateInterface> AR3HardwareInterface::export_state_interfaces()
{
  // TODO: Implement export_state_interfaces
  return std::vector<hardware_interface::StateInterface>();
}

std::vector<hardware_interface::CommandInterface> AR3HardwareInterface::export_command_interfaces()
{
  // TODO: Implement export_command_interfaces
  return std::vector<hardware_interface::CommandInterface>();
}

rclcpp::Logger AR3HardwareInterface::get_logger() const
{
  return rclcpp::get_logger("ar3_hardware_interface");
}

}  // namespace ar3_hardware_interface