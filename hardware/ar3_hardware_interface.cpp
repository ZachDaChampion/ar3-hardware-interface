#include "ar3_hardware_interface/ar3_hardware_interface.hpp"

#define FW_VERSION "1"

namespace asio = boost::asio;

namespace ar3_hardware_interface
{

ar3_hardware_interface::AR3HardwareInterface::AR3HardwareInterface()
  : io_service_(), serial_port_(io_service_)
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

  this->serial_dev_ = info_.hardware_parameters["serial_port"];
  this->baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);

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
    this->serial_port_.set_option(asio::serial_port_base::baud_rate(this->baud_rate_));
    this->serial_port_.open(this->serial_dev_);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(this->get_logger(), "Opened serial port");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
AR3HardwareInterface::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  // Close serial port
  try {
    this->serial_port_.close();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to close serial port: %s", e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(this->get_logger(), "Closed serial port");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
AR3HardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  // Send start command with firmware version
  char write_buf[8];  // Space for "STA" + 3 digit version + newline + null terminator
  snprintf(write_buf, sizeof(write_buf), "STA%s\n", FW_VERSION);
  try {
    asio::write(this->serial_port_, asio::buffer(write_buf, sizeof(write_buf)));
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to write to serial port: %s", e.what());
    return CallbackReturn::ERROR;
  }

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