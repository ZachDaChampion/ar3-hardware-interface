/**
 * @file messaging.hpp
 * @author Zach Champion (zachchampion79@gmail.com)
 *
 * @version 1.0
 * @date 2023-09-25
 */

#ifndef AR3_HARDWARE_INTERFACE__MESSAGING_HPP
#define AR3_HARDWARE_INTERFACE__MESSAGING_HPP

#include <libserial/SerialPort.h>

#include <boost/optional.hpp>
#include <deque>
#include <list>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

/**
 * Errors that can be returned by the robot.
 */
class CobotError
{
public:
  enum class Code {
    OTHER = 0,
    MALFORMED_REQUEST = 1,
    OUT_OF_RANGE = 2,
    INVALID_JOINT = 3,
    NOT_INITIALIZED = 4,
    NOT_CALIBRATED = 5,
    CANCELLED = 6,
    INVALID_FIRMWARE_VERSION = 7,
  };

  Code code_;
  std::string message_;

  /**
   * Construct a new CobotError object.
   *
   * @param[in] code The error code.
   * @param[in] message The error message.
   */
  CobotError(uint8_t code, std::string message);

  /**
   * Return a string representation of the error.
   *
   * @return A string representation of the error.
   */
  std::string to_string() const;
};

/**
 * The log level of a log message.
 */
enum class LogLevel {
  DEBUG = 0,  // The lowest log level. Used for debugging. Should not be used in production.
  INFO = 1,   // Used for general information.
  WARN = 2,   // Used for warnings.
  ERROR = 3,  // Used for errors.
  NONE = 4    // Used to disable logging.
};

/**
 * The type of a request message.
 */
enum class RequestType {
  Init = 0,
  Calibrate = 1,
  Override = 2,
  GetJoints = 3,
  MoveTo = 4,
  MoveSpeed = 5,
  Stop = 6,
  GoHome = 7,
  Reset = 8,
  SetLogLevel = 9,
  SetFeedback = 10,
};

/**
 * The type of an received message.
 */
enum class ReceivedMessageType {
  Log = 0,
  Response = 1,
};

/**
 * A response message that has been received.
 */
struct Response {
  /**
   * The type of a response.
   *
   */
  enum class Type {
    Ack = 0,
    Done = 1,
    Error = 2,
    Joints = 3,
  };

  uint32_t uuid;
  Type type;
  boost::optional<std::vector<uint8_t>> data;
};

class Messenger
{
public:
  /**
   * Construct a new Messenger object.
   */
  Messenger();

  Messenger(const Messenger&) = delete;
  Messenger& operator=(const Messenger&) = delete;

  /**
   * Blocks until a response is received with the given ID.
   *
   * @param[in] msg_id The ID of the response to look for.
   * @param[in] serial_port The serial port to listen with.
   * @param[in] logger The logger to log messages to.
   * @param[in] timeout The maximum time to wait for a response, in milliseconds. If 0, this will
   *                    wait forever.
   * @return The response that had a matching ID.
   */
  std::unique_ptr<Response> wait_for_response(uint32_t msg_id, LibSerial::SerialPort& serial_port,
                                              rclcpp::Logger& logger, uint timeout = 0);

  /**
   * Blocks until an acknowledgment is received for the given message ID. If an error is received
   * or the timeout is exceeded, it will be logged and an exception will be thrown.
   *
   * @param[in] msg_id The ID to look for ACKs from.
   * @param[in] serial_port The serial port to listen with.
   * @param[in] logger The logger to log messages to.
   * @param[in] timeout The maximum time to wait for a response, in milliseconds. If 0, this will
   *                    wait forever.
   */
  void wait_for_ack(uint32_t msg_id, LibSerial::SerialPort& serial_port, rclcpp::Logger& logger,
                    uint timeout = 0);

  /**
   * Blocks until a done message is received for the given message ID. If an error is received or
   * the timeout is exceeded, it will be logged and an exception will be thrown.
   *
   * @param[in] msg_id The ID to look for DONEs from.
   * @param[in] serial_port The serial port to listen with.
   * @param[in] logger The logger to log messages to.
   * @param[in] timeout The maximum time to wait for a response, in milliseconds. If 0, this will
   *                    wait forever.
   */
  void wait_for_done(uint32_t msg_id, LibSerial::SerialPort& serial_port, rclcpp::Logger& logger,
                     uint timeout = 0);

  /**
   * Send a request to the COBOT.
   *
   * @param[in] type The type of request to send.
   * @param[in] payload The payload of the request.
   * @param[in] payload_len The length of the payload.
   * @param[in] serial_port The serial port to send the request to.
   * @param[in] logger The logger to log messages to.
   * @return The ID of the message that was sent.
   */
  uint32_t send_request(RequestType type, const uint8_t* payload, size_t payload_len,
                        LibSerial::SerialPort& serial_port, rclcpp::Logger& logger);

private:
  /**
   * Try to receive a message from the serial port. This will read all available bytes from the
   * port into `in_buffer_`, then parse any messages that can be found in the buffer. Log messages
   * will be forwarded to the logger and otherwise ignored. Response messages will be added to the
   * response queue.
   *
   * @param[in] serial_port The serial port to receive from.
   * @param[in] logger The logger to forward log messages to.
   * @return True if a non-log message was received, false otherwise.
   */
  bool try_recv(LibSerial::SerialPort& serial_port, rclcpp::Logger& logger);

  /**
   * Send a message to the COBOT. This will prepend the message with an appropriate header.
   *
   * @param[in] msg The message to send.
   * @param[in] msg_len The length of the message.
   * @param[in] serial_port The serial port to send the message to.
   * @param[in] logger The logger to log messages to.
   */
  void send_msg(const uint8_t* msg, size_t msg_len, LibSerial::SerialPort& serial_port,
                rclcpp::Logger& logger);

  /**
   * Generates a new UUID for a message. This is a 32-bit number used to identify messages. The
   * first 16 bits correspond to the current millisecond of epoch time, and the last 16 bits are
   * incremented for each message.
   *
   * This system is used to prevent duplicate messages from being sent to the robot. The
   * incrementing counter is used to prevent duplicate messages from being sent in the same
   * millisecond, and the millisecond time is used to prevent duplicate messages from being sent
   * when the node is restarted, or when the counter overflows.
   *
   * @return a new UUID
   */
  uint32_t gen_uuid();

  /**
   * If the given message is an error, parse it and throw an exception.
   *
   * @param[in] response The response to check.
   * @param[in] logger The logger to log messages to.
   */
  void throw_if_error(const Response& response, rclcpp::Logger& logger);

  // Internal buffers for incoming and outgoing messages.
  std::deque<uint8_t> in_buffer_;
  std::vector<uint8_t> out_buffer_;

  // A list of response messages that have been received.
  std::list<std::unique_ptr<Response>> response_queue_;

  // Total number of messages sent.
  uint32_t msg_count_;
};

#endif