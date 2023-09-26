/**
 * @file ar3_hardware_interface.hpp
 * @author Zach Champion (zachchampion79@gmail.com)
 *
 * @version 1.0
 * @date 2023-09-25
 *
 *
 * This header defines the AR3HardwareInterface class. This is a hardware interface for
 * `ros2_control` that allows the AR3 robot to be controlled by a controller manager.
 *
 * The documentation for `ros2_control` is somewhat unclear on how to implement hardware
 * interfaces. See https://youtu.be/J02jEKawE5U for a good explanation of how the
 * system works.
 *
 * In short, each hardware interface must inherit from one of the following classes:
 *
 * - `hardware_interface::SystemInterface`
 * - `hardware_interface::ActuatorInterface`
 * - `hardware_interface::SensorInterface`
 *
 * A hardware interface implements a state machine that controls its lifecycle:
 *
 *       on_init()                    on_shutdown()
 *          │      ┌────────────────┐      ▲
 *          └─────►│                ├──────┘
 *                 │  Unconfigured  │
 *          ┌──────┤                │◄─────┐
 *          │      └────────────────┘      │
 *  on_configure()                    on_cleanup()
 *          │      ┌────────────────┐      │
 *          └─────►│                ├──────┘
 *                 │    Inactive    │
 *          ┌──────┤                │◄─────┐
 *          │      └────────────────┘      │
 *   on_activate()                    on_deactivate()
 *          │      ┌────────────────┐      │
 *          └─────►│                ├──────┘
 *                 │     Active     │
 *                 │                │
 *                 └────────────────┘
 *
 *                  ┌─── read()◄──┐
 *                  │             │
 *                  └──► write()──┘
 */

#ifndef AR3_HARDWARE_INTERFACE__AR3_HARDWARE_INTERFACE_HPP
#define AR3_HARDWARE_INTERFACE__AR3_HARDWARE_INTERFACE_HPP

#include <libserial/SerialPort.h>

#include "ar3_hardware_interface/messaging.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ar3_hardware_interface
{
class AR3HardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AR3HardwareInterface)

  /**
   * Construct a new AR3HardwareInterface object.
   */
  AR3HardwareInterface();

  /**
   * Initialization of the hardware interface from data parsed from the robot's URDF.
   * The hardware interface is expected to be in the "unconfigured" state after this call.
   *
   * \param[in] hardware_info structure with data from URDF.
   * \returns CallbackReturn::SUCCESS if required data are provided and can be parsed.
   * \returns CallbackReturn::ERROR if any error happens or data are missing.
   */
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  /**
   * Gracefully shutdown the hardware interface. Free any resources that were allocated.
   *
   * \param[in] previous_state the state the hardware interface was previously in.
   * \returns CallbackReturn::SUCCESS if the shutdown was successful.
   * \returns CallbackReturn::ERROR if there was an error during shutdown.
   */
  hardware_interface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * Configure the hardware interface. This should involve opening connections to the
   * hardware and configuring it. After this call, the hardware interface should be
   * in the "inactive" state.
   *
   * \param[in] previous_state the state the hardware interface was previously in.
   * \return CallbackReturn::SUCCESS if the hardware interface was configured successfully.
   * \return CallbackReturn::ERROR if there was an error during configuration.
   */
  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * Cleanup the hardware interface. This should involve closing connections to the
   * hardware.
   *
   * \param[in] previous_state the state the hardware interface was previously in.
   * \return CallbackReturn::SUCCESS if the hardware interface was cleaned up successfully.
   * \return CallbackReturn::ERROR if there was an error during cleanup.
   */
  hardware_interface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * Activate the hardware interface. Here, you should engage actuators and enable
   * sensors. After this is run, the hardware interface will be in the "active" state.
   *
   * \param[in] previous_state the state the hardware interface was in before this
   * \return return code
   */
  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * Deactivate the hardware interface. Here, you should disengage actuators and disable
   * sensors. After this is run, the hardware interface will be in the "inactive" state.
   *
   * \param[in] previous_state the state the hardware interface was in before this
   * \return return code
   */
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * Read the current state values from the actuator.
   *
   * The data readings from the physical hardware has to be updated
   * and reflected accordingly in the exported state interfaces.
   * That is, the data pointed by the interfaces shall be updated.
   *
   * \param[in] time The time at the start of this control loop iteration
   * \param[in] period The measured time taken by the last control loop iteration
   * \return return_type::OK if the read was successful, return_type::ERROR otherwise.
   */
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  /**
   * Write the current command values to the actuator.
   *
   * The physical hardware shall be updated with the latest value from
   * the exported command interfaces.
   *
   * \param[in] time The time at the start of this control loop iteration
   * \param[in] period The measured time taken by the last control loop iteration
   * \return return_type::OK if the read was successful, return_type::ERROR otherwise.
   */
  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;

  /**
   * Exports all state interfaces for this hardware interface.
   *
   * The state interfaces have to be created and transferred according
   * to the hardware info passed in for the configuration.
   *
   * Note the ownership over the state interfaces is transferred to the caller.
   *
   * \return vector of state interfaces.
   */
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /**
   * Exports all command interfaces for this hardware interface.
   *
   * The command interfaces have to be created and transferred according
   * to the hardware info passed in for the configuration.
   *
   * Note the ownership over the state interfaces is transferred to the caller.
   *
   * \return vector of command interfaces.
   */
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /**
   * Get the logger name of the hardware interface.
   *
   * \return logger name.
   */
  rclcpp::Logger get_logger() const;

private:
  // Serial port parameters.
  LibSerial::BaudRate baud_rate_;
  std::string serial_dev_name_;

  // Serial port object.
  LibSerial::SerialPort serial_port_;

  // Messenger object for sending and receiving messages.
  Messenger messenger_;
};
}  // namespace ar3_hardware_interface

#endif
