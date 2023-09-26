/**
 * @file checksum.hpp
 * @author Zach Champion (zachchampion79@gmail.com)
 *
 * @version 1.0
 * @date 2023-07-09
 */

#ifndef AR3_TEENSY_FIRMWARE__CHECKSUM_H
#define AR3_TEENSY_FIRMWARE__CHECKSUM_H

#include <stddef.h>
#include <stdint.h>

/**
 * Calculate the CRC-8 checksum of a message. This checksum uses the polynomial `0x7`, with an
 * initial value of `0x0` and no final XOR value.
 *
 * @param[in] data The message to calculate the checksum of.
 * @param[in] len The length of the message.
 * @return The CRC-8 checksum of the message.
 */
uint8_t crc8ccitt(const void* data, size_t size);

/**
 * Check the CRC-8 checksum of a message. This checksum uses the polynomial `0x7`, with an initial
 * value of `0x0` and no final XOR value.
 *
 * @param[in] data The message to calculate the checksum of.
 * @param[in] len The length of the message.
 * @param[in] checksum The checksum to check against.
 * @return `true` if the checksum matches, `false` otherwise.
 */
bool crc8ccitt_check(const void* data, size_t size, uint8_t checksum);

#endif  // AR3_TEENSY_FIRMWARE__CHECKSUM_H