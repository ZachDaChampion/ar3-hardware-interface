/**
 * @file ar3_hardware_interface.cpp
 * @author Zach Champion (zachchampion79@gmail.com)
 *
 * @version 1.0
 * @date 2023-09-25
 */

#include "ar3_hardware_interface/ar3_hardware_interface.hpp"
#include "ar3_hardware_interface/checksum.hpp"
#include <chrono>

#define FW_VERSION 3

//                                                                                                //
// ===================================== Utility functions ====================================== //
//                                                                                                //

/**
 * Generates a new UUID for a message. This is a 32-bit number used to identify messages. The first
 * 16 bits correspond to the current millisecond of epoch time, and the last 16 bits are incremented
 * for each message.
 *
 * This system is used to prevent duplicate messages from being sent to the robot. The incrementing
 * counter is used to prevent duplicate messages from being sent in the same millisecond, and the
 * millisecond time is used to prevent duplicate messages from being sent when the node is
 * restarted, or when the counter overflows.
 *
 * @return a new UUID
 */
static uint32_t gen_uuid()
{
  static uint16_t msg_count = 0;

  auto now = std::chrono::system_clock::now();
  auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
  auto value = now_ms.time_since_epoch();

  uint16_t uuid_time = value.count() & 0xFFFF;
  uint16_t uuid_count = msg_count++;
  return (uuid_count << 16) | uuid_time;
}

//                                                                                                //
// ===================================== Hardware interface ===================================== //
//                                                                                                //

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
  // TODO: Implement on_shutdown
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
  // Send start command with firmware version
  // uint8_t write_buf[3 + 5 + 4];  // 3 byte fame, 5 byte request, 4 byte payload
  // write_buf[0] = START_BYTE;
  // write_buf[3] = static_cast<uint8_t>(Request::Init);
  // try {
  //   asio::write(this->serial_port_, asio::buffer(write_buf, sizeof(write_buf)));
  // } catch (const std::exception& e) {
  //   RCLCPP_ERROR(this->get_logger(), "Failed to write to serial port: %s", e.what());
  //   return CallbackReturn::ERROR;
  // }

  // TODO: Wait for response from robot

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
AR3HardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  // TODO: Implement on_deactivate
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