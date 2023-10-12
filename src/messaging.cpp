/**
 * @file messaging.cpp
 * @author Zach Champion (zachchampion79@gmail.com)
 *
 * @version 1.0
 * @date 2023-09-25
 */

#include "ar3_hardware_interface/messaging.hpp"

#include <chrono>
#include <sstream>

#include "ar3_hardware_interface/checksum.hpp"
#include "ar3_hardware_interface/serialize.h"

using namespace std;
using namespace chrono;
using namespace LibSerial;

#define MSG_START_BYTE 0x24

CobotError::CobotError(uint8_t code, string message) : message_(message)
{
  switch (code) {
    case static_cast<uint8_t>(Code::OTHER):
      code_ = Code::OTHER;
      break;
    case static_cast<uint8_t>(Code::MALFORMED_REQUEST):
      code_ = Code::MALFORMED_REQUEST;
      break;
    case static_cast<uint8_t>(Code::OUT_OF_RANGE):
      code_ = Code::OUT_OF_RANGE;
      break;
    case static_cast<uint8_t>(Code::INVALID_JOINT):
      code_ = Code::INVALID_JOINT;
      break;
    case static_cast<uint8_t>(Code::NOT_INITIALIZED):
      code_ = Code::NOT_INITIALIZED;
      break;
    case static_cast<uint8_t>(Code::NOT_CALIBRATED):
      code_ = Code::NOT_CALIBRATED;
      break;
    case static_cast<uint8_t>(Code::CANCELLED):
      code_ = Code::CANCELLED;
      break;
    case static_cast<uint8_t>(Code::INVALID_FIRMWARE_VERSION):
      code_ = Code::INVALID_FIRMWARE_VERSION;
      break;

    default:
      code_ = Code::OTHER;
      break;
  }
}

string CobotError::to_string() const
{
  stringstream ss;
  ss << "COBOT error: ";
  switch (code_) {
    case Code::OTHER:
      ss << "(other/unknown)";
      break;
    case Code::MALFORMED_REQUEST:
      ss << "(malformed request)";
      break;
    case Code::OUT_OF_RANGE:
      ss << "(out of range)";
      break;
    case Code::INVALID_JOINT:
      ss << "(invalid joint)";
      break;
    case Code::NOT_INITIALIZED:
      ss << "(not initialized)";
      break;
    case Code::NOT_CALIBRATED:
      ss << "(not calibrated)";
      break;
    case Code::CANCELLED:
      ss << "(cancelled)";
      break;
    case Code::INVALID_FIRMWARE_VERSION:
      ss << "(invalid firmware version)";
      break;
  }
  if (!message_.empty()) ss << " " << message_;
  return ss.str();
}

Messenger::Messenger(int32_t expire_response_after)
  : msg_count_(0), expire_responses_after_(expire_response_after)
{
}

unique_ptr<Response> Messenger::wait_for_response(uint32_t msg_id, SerialPort& serial_port,
                                                  rclcpp::Logger& logger, uint timeout)
{
  auto start_time = steady_clock::now();
  while (true) {
    // Check if the timeout has been exceeded.
    if (timeout > 0) {
      auto elapsed = steady_clock::now() - start_time;
      if (duration_cast<milliseconds>(elapsed).count() > timeout) {
        RCLCPP_WARN(logger, "Timeout exceeded");
        throw runtime_error("Timeout exceeded");
      }
    }

    // Delete all expired responses.
    auto current_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    for (auto it = response_queue_.begin(); it != response_queue_.end();) {
      auto expiration_time = (*it)->expiration_time;
      if (expiration_time != 0 && current_time > expiration_time) {
        it = response_queue_.erase(it);
      } else {
        ++it;
      }
    }

    // Check if a response has been received.
    for (auto it = response_queue_.begin(); it != response_queue_.end(); ++it) {
      if ((*it)->uuid == msg_id) {
        auto response = move(*it);
        response_queue_.erase(it);
        return response;
      }
    }

    // If no response has been received, try to receive one.
    if (!try_recv(serial_port, logger)) {
      this_thread::sleep_for(milliseconds(1));
    }
  }
}

void Messenger::wait_for_ack(uint32_t msg_id, LibSerial::SerialPort& serial_port,
                             rclcpp::Logger& logger, uint timeout)
{
  auto start_time = steady_clock::now();
  while (true) {
    // Check if the timeout has been exceeded.
    uint time_left = 0;
    if (timeout > 0) {
      auto elapsed = steady_clock::now() - start_time;
      auto elapsed_ms = duration_cast<milliseconds>(elapsed).count();
      if (elapsed_ms > timeout) {
        RCLCPP_WARN(logger, "Timeout exceeded");
        throw runtime_error("Timeout exceeded");
      }
      time_left = timeout - elapsed_ms;
    }

    // Wait for a response and check if it's an ACK or an error.
    auto response = wait_for_response(msg_id, serial_port, logger, time_left);
    if (response->type == Response::Type::Ack) return;
    throw_if_error(*response, logger);

    // If the response is not an ACK or an error, log it and continue.
    RCLCPP_WARN(logger, "Unexpected response type '%d' while waiting for ACK",
                static_cast<int>(response->type));
  }
}

void Messenger::wait_for_done(uint32_t msg_id, LibSerial::SerialPort& serial_port,
                              rclcpp::Logger& logger, uint timeout)
{
  auto start_time = steady_clock::now();
  while (true) {
    // Check if the timeout has been exceeded.
    uint time_left = 0;
    if (timeout > 0) {
      auto elapsed = steady_clock::now() - start_time;
      auto elapsed_ms = duration_cast<milliseconds>(elapsed).count();
      if (elapsed_ms > timeout) {
        RCLCPP_WARN(logger, "Timeout exceeded");
        throw runtime_error("Timeout exceeded");
      }
      time_left = timeout - elapsed_ms;
    }

    // Wait for a response and check if it's a DONE or an error.
    auto response = wait_for_response(msg_id, serial_port, logger, time_left);
    if (response->type == Response::Type::Done) return;
    throw_if_error(*response, logger);

    // If the response is not a DONE or an error, log it and continue.
    RCLCPP_WARN(logger, "Unexpected response type '%d' while waiting for DONE",
                static_cast<int>(response->type));
  }
}

uint32_t Messenger::send_request(RequestType type, const uint8_t* payload, size_t payload_len,
                                 LibSerial::SerialPort& serial_port, rclcpp::Logger& logger)
{
  // Generate a UUID for the message.
  uint32_t uuid = gen_uuid();

  // Construct the message.
  size_t msg_len = 1 + 4 + payload_len;  // 1 byte type, 4 bytes UUID, payload
  auto msg = new uint8_t[msg_len];
  msg[0] = static_cast<uint8_t>(type);
  serialize_uint32(msg + 1, msg_len - 1, uuid);
  if (payload_len > 0) memcpy(msg + 5, payload, payload_len);

  // Send the message.
  try {
    send_msg(msg, msg_len, serial_port, logger);
  } catch (const exception& e) {
    delete[] msg;
    throw e;
  }

  // Clean up and return the UUID.
  delete[] msg;
  return uuid;
}

bool Messenger::try_recv(LibSerial::SerialPort& serial_port, rclcpp::Logger& logger)
{
  // Read all available bytes from the serial port into the buffer.
  char next_byte;
  while (serial_port.IsDataAvailable()) {
    serial_port.ReadByte(next_byte);
    in_buffer_.push_back(next_byte);
  }

  // Parse messages from the buffer.
  size_t cursor = 0;
  size_t shift_by = 0;
  bool found_message = false;
  while (cursor < in_buffer_.size()) {
    // Move the cursor and shift the buffer until the start of a message is found.
    while (cursor < in_buffer_.size() && in_buffer_[cursor] != MSG_START_BYTE) {
      ++cursor;
      ++shift_by;
    }

    // Read the header and verify that it fits in the buffer.
    if (in_buffer_.size() - cursor < 3u) {
      in_buffer_.erase(in_buffer_.begin(), in_buffer_.begin() + shift_by);
      return false;
    }
    uint8_t payload_size = in_buffer_[cursor + 1];
    uint8_t crc = in_buffer_[cursor + 2];

    // Verify that the payload fits in the buffer.
    if (in_buffer_.size() - cursor < 3u + payload_size) {
      in_buffer_.erase(in_buffer_.begin(), in_buffer_.begin() + shift_by);
      return false;
    }

    // From now on, we can assume that the message is complete. If this loop ever continues, we
    // want to resume after the end of this message.
    size_t payload = cursor + 3;
    cursor += 3 + payload_size;
    shift_by += 3 + payload_size;

    // Verify the payload's CRC.
    if (!crc8ccitt_check(&in_buffer_[payload], payload_size, crc)) {
      RCLCPP_WARN(logger, "CRC failed");
      continue;
    }

    // Parse the payload.
    uint8_t msg_type = in_buffer_[payload];
    switch (msg_type) {
      // If the payload is a log message, log it and continue.
      case static_cast<uint8_t>(ReceivedMessageType::Log): {
        if (payload_size < 2) {
          RCLCPP_WARN(logger, "Malformed COBOT log message");
          continue;
        }

        uint8_t log_level = in_buffer_[payload + 1];
        uint8_t log_str_len = in_buffer_[payload + 2];

        if (payload_size < 3 + log_str_len) {
          RCLCPP_WARN(logger, "COBOT log message (%u) longer than payload (%u)", 3 + log_str_len,
                      payload_size);
          continue;
        }

        string log_str(&in_buffer_[payload + 3], &in_buffer_[payload + 3 + log_str_len]);

        switch (log_level) {
          case static_cast<uint8_t>(LogLevel::DEBUG):
            RCLCPP_DEBUG(logger, "COBOT: %s", log_str.c_str());
            break;
          case static_cast<uint8_t>(LogLevel::INFO):
            RCLCPP_INFO(logger, "COBOT: %s", log_str.c_str());
            break;
          case static_cast<uint8_t>(LogLevel::WARN):
            RCLCPP_WARN(logger, "COBOT: %s", log_str.c_str());
            break;
          case static_cast<uint8_t>(LogLevel::ERROR):
            RCLCPP_ERROR(logger, "COBOT: %s", log_str.c_str());
            break;

          default:
            RCLCPP_WARN(logger, "COBOT log with unknown level '%02x': %s", log_level,
                        log_str.c_str());
            break;
        }
      } break;

      // If the payload is a response message, parse it and return it.
      case static_cast<uint8_t>(ReceivedMessageType::Response): {
        if (payload_size < 6) {
          RCLCPP_WARN(logger, "Malformed COBOT response message");
          continue;
        }

        auto response = make_unique<Response>();

        // Set the response's expiration time.
        auto current_time =
            duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
        response->expiration_time = current_time + expire_responses_after_;

        // Parse the response's type.
        switch (in_buffer_[payload + 1]) {
          case static_cast<uint8_t>(Response::Type::Ack):
            response->type = Response::Type::Ack;
            break;
          case static_cast<uint8_t>(Response::Type::Done):
            response->type = Response::Type::Done;
            break;
          case static_cast<uint8_t>(Response::Type::Error):
            response->type = Response::Type::Error;
            break;
          case static_cast<uint8_t>(Response::Type::Joints):
            response->type = Response::Type::Joints;
            break;

          default:
            RCLCPP_WARN(logger, "COBOT response with unknown type '%02x'", in_buffer_[payload + 1]);
            continue;
        }

        // Parse the response's UUID.
        deserialize_uint32(&response->uuid, &in_buffer_[payload + 2]);

        // Parse the response's data.
        if (payload_size > 6)
          response->data =
              vector<uint8_t>(&in_buffer_[payload + 6], &in_buffer_[payload + payload_size]);
        else
          response->data = boost::none;

        // Add the response to the queue.
        response_queue_.push_back(move(response));
        found_message = true;
      } break;
    }
  }

  // Shift the buffer.
  if (shift_by > 0) {
    in_buffer_.erase(in_buffer_.begin(), in_buffer_.begin() + shift_by);
  }

  return found_message;
}

void Messenger::send_msg(const uint8_t* msg, size_t msg_len, LibSerial::SerialPort& serial_port,
                         rclcpp::Logger& logger)
{
  // Calculate the CRC.
  uint8_t crc = crc8ccitt(msg, msg_len);

  // Construct the header.
  out_buffer_.clear();
  out_buffer_.push_back(MSG_START_BYTE);
  out_buffer_.push_back(static_cast<uint8_t>(msg_len));
  out_buffer_.push_back(crc);

  // Append the payload.
  out_buffer_.insert(out_buffer_.end(), msg, msg + msg_len);

  // Send the message to the serial port.
  try {
    serial_port.Write(out_buffer_);
  } catch (const exception& e) {
    RCLCPP_ERROR(logger, "Failed to write to serial port: %s", e.what());
    throw e;
  }
}

uint32_t Messenger::gen_uuid()
{
  auto now = system_clock::now();
  auto now_ms = time_point_cast<milliseconds>(now);
  auto value = now_ms.time_since_epoch();

  uint16_t uuid_time = value.count() & 0xFFFF;
  uint16_t uuid_count = msg_count_++;
  return (uuid_count << 16) | uuid_time;
}

void Messenger::throw_if_error(const Response& response, rclcpp::Logger& logger)
{
  if (response.type != Response::Type::Error) return;

  if (!response.data) {
    RCLCPP_WARN(logger, "COBOT error: (no data)");
    throw runtime_error("COBOT error: (no data)");
  }
  if (response.data->size() < 2) {
    RCLCPP_WARN(logger, "COBOT error: (data too short)");
    throw runtime_error("COBOT error: (data too short)");
  }

  uint8_t error_code = (*response.data)[0];
  uint8_t error_msg_len = (*response.data)[1];

  if (response.data->size() < 2u + error_msg_len) {
    RCLCPP_WARN(logger, "COBOT error: (data too short)");
    throw runtime_error("COBOT error: (data too short)");
  }

  string error_msg(&(*response.data)[2], &(*response.data)[2 + error_msg_len]);
  RCLCPP_ERROR(logger, "COBOT error: %s", error_msg.c_str());
  throw CobotError(error_code, error_msg);
}