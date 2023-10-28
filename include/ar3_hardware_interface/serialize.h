/**
 * @file serialize.h
 * @author Zach Champion (zachchampion79@gmail.com)
 *
 * @version 2.0
 * @date 2023-09-19
 */

#ifndef AR3_TEENSY_FIRMWARE__SERIALIZE_H
#define AR3_TEENSY_FIRMWARE__SERIALIZE_H

#include <stdint.h>
#include <stddef.h>

/**
 * Serialize a int8 into a buffer.
 *
 * @param dest The destination buffer for the serialized int8.
 * @param dest_size The size of the destination buffer.
 * @param val The int8 to serialize.
 * @return The number of bytes written to the destination buffer or -1 if the buffer is too small.
 */
int serialize_int8(uint8_t* const dest, const size_t dest_size, const int8_t val);

/**
 * Serialize a int16 into a buffer.
 *
 * @param dest The destination buffer for the serialized int16.
 * @param dest_size The size of the destination buffer.
 * @param val The int16 to serialize.
 * @return The number of bytes written to the destination buffer or -1 if the buffer is too small.
 */
int serialize_int16(uint8_t* const dest, const size_t dest_size, const int16_t val);

/**
 * Serialize a int32 into a buffer.
 *
 * @param dest The destination buffer for the serialized int32.
 * @param dest_size The size of the destination buffer.
 * @param val The int32 to serialize.
 * @return The number of bytes written to the destination buffer or -1 if the buffer is too small.
 */
int serialize_int32(uint8_t* const dest, const size_t dest_size, const int32_t val);

/**
 * Serialize a int64 into a buffer.
 *
 * @param dest The destination buffer for the serialized int64.
 * @param dest_size The size of the destination buffer.
 * @param val The int64 to serialize.
 * @return The number of bytes written to the destination buffer or -1 if the buffer is too small.
 */
int serialize_int64(uint8_t* const dest, const size_t dest_size, const int64_t val);

/**
 * Serialize a uint8 into a buffer.
 *
 * @param dest The destination buffer for the serialized int8.
 * @param dest_size The size of the destination buffer.
 * @param val The int8 to serialize.
 * @return The number of bytes written to the destination buffer or -1 if the buffer is too small.
 */
int serialize_uint8(uint8_t* const dest, const size_t dest_size, const uint8_t val);

/**
 * Serialize a uint16 into a buffer.
 *
 * @param dest The destination buffer for the serialized int16.
 * @param dest_size The size of the destination buffer.
 * @param val The int16 to serialize.
 * @return The number of bytes written to the destination buffer or -1 if the buffer is too small.
 */
int serialize_uint16(uint8_t* const dest, const size_t dest_size, const uint16_t val);

/**
 * Serialize a uint32 into a buffer.
 *
 * @param dest The destination buffer for the serialized int32.
 * @param dest_size The size of the destination buffer.
 * @param val The int32 to serialize.
 * @return The number of bytes written to the destination buffer or -1 if the buffer is too small.
 */
int serialize_uint32(uint8_t* const dest, const size_t dest_size, const uint32_t val);

/**
 * Serialize a uint64 into a buffer.
 *
 * @param dest The destination buffer for the serialized int64.
 * @param dest_size The size of the destination buffer.
 * @param val The int64 to serialize.
 * @return The number of bytes written to the destination buffer or -1 if the buffer is too small.
 */
int serialize_uint64(uint8_t* const dest, const size_t dest_size, const uint64_t val);

/**
 * Serialize a float into a buffer.
 *
 * @param dest The destination buffer for the serialized float.
 * @param dest_size The size of the destination buffer.
 * @param val The float to serialize.
 * @return The number of bytes written to the destination buffer or -1 if the buffer is too small.
 */
int serialize_float(uint8_t* const dest, const size_t dest_size, const float val);

/**
 * Serialize a double into a buffer.
 *
 * @param dest The destination buffer for the serialized double.
 * @param dest_size The size of the destination buffer.
 * @param val The double to serialize.
 * @return The number of bytes written to the destination buffer or -1 if the buffer is too small.
 */
int serialize_double(uint8_t* const dest, const size_t dest_size, const double val);

/**
 * Serialize a string into a buffer.
 *
 * @param dest The destination buffer for the serialized string.
 * @param dest_size The size of the destination buffer.
 * @param str The string to serialize.
 * @return The number of bytes written to the destination buffer or -1 if the buffer is too small.
 */
int serialize_string(uint8_t* const dest, const size_t dest_size, const char* const str);

/**
 * Serialize a buffer into a buffer.
 *
 * @param dest The destination buffer for the serialized buffer.
 * @param dest_size The size of the destination buffer.
 * @param buf The buffer to serialize.
 * @param buf_size The size of the buffer to serialize.
 * @return The number of bytes written to the destination buffer or -1 if the buffer is too small.
 */
int serialize_buffer(uint8_t* const dest, const size_t dest_size, const uint8_t* const buf,
                     const size_t buf_size);

/**
 * Deserialize a int8 from a buffer.
 *
 * @param dest The destination buffer for the deserialized int8.
 * @param dest_size The size of the destination buffer.
 * @param src The source buffer for the deserialized int8.
 * @return The number of bytes read from the source buffer or -1 if the buffer is too small.
 */
int deserialize_int8(int8_t* const dest, const uint8_t* const src);

/**
 * Deserialize a int16 from a buffer.
 *
 * @param dest The destination buffer for the deserialized int16.
 * @param dest_size The size of the destination buffer.
 * @param src The source buffer for the deserialized int16.
 * @return The number of bytes read from the source buffer or -1 if the buffer is too small.
 */
int deserialize_int16(int16_t* const dest, const uint8_t* const src);

/**
 * Deserialize a int32 from a buffer.
 *
 * @param dest The destination buffer for the deserialized int32.
 * @param dest_size The size of the destination buffer.
 * @param src The source buffer for the deserialized int32.
 * @return The number of bytes read from the source buffer or -1 if the buffer is too small.
 */
int deserialize_int32(int32_t* const dest, const uint8_t* const src);

/**
 * Deserialize a int64 from a buffer.
 *
 * @param dest The destination buffer for the deserialized int64.
 * @param dest_size The size of the destination buffer.
 * @param src The source buffer for the deserialized int64.
 * @return The number of bytes read from the source buffer or -1 if the buffer is too small.
 */
int deserialize_int64(int64_t* const dest, const uint8_t* const src);

/**
 * Deserialize a uint8 from a buffer.
 *
 * @param dest The destination buffer for the deserialized uint8.
 * @param dest_size The size of the destination buffer.
 * @param src The source buffer for the deserialized uint8.
 * @return The number of bytes read from the source buffer or -1 if the buffer is too small.
 */
int deserialize_uint8(uint8_t* const dest, const uint8_t* const src);

/**
 * Deserialize a uint16 from a buffer.
 *
 * @param dest The destination buffer for the deserialized uint16.
 * @param dest_size The size of the destination buffer.
 * @param src The source buffer for the deserialized uint16.
 * @return The number of bytes read from the source buffer or -1 if the buffer is too small.
 */
int deserialize_uint16(uint16_t* const dest, const uint8_t* const src);

/**
 * Deserialize a uint32 from a buffer.
 *
 * @param dest The destination buffer for the deserialized uint32.
 * @param dest_size The size of the destination buffer.
 * @param src The source buffer for the deserialized uint32.
 * @return The number of bytes read from the source buffer or -1 if the buffer is too small.
 */
int deserialize_uint32(uint32_t* const dest, const uint8_t* const src);

/**
 * Deserialize a uint64 from a buffer.
 *
 * @param dest The destination buffer for the deserialized uint64.
 * @param dest_size The size of the destination buffer.
 * @param src The source buffer for the deserialized uint64.
 * @return The number of bytes read from the source buffer or -1 if the buffer is too small.
 */
int deserialize_uint64(uint64_t* const dest, const uint8_t* const src);

/**
 * Deserialize a float from a buffer.
 *
 * @param dest The destination buffer for the deserialized float.
 * @param dest_size The size of the destination buffer.
 * @param src The source buffer for the deserialized float.
 * @return The number of bytes read from the source buffer or -1 if the buffer is too small.
 */
int deserialize_float(float* const dest, const uint8_t* const src);

/**
 * Deserialize a double from a buffer.
 *
 * @param dest The destination buffer for the deserialized double.
 * @param dest_size The size of the destination buffer.
 * @param src The source buffer for the deserialized double.
 * @return The number of bytes read from the source buffer or -1 if the buffer is too small.
 */
int deserialize_double(double* const dest, const uint8_t* const src);

/**
 * Deserialize a string from a buffer.
 *
 * @param dest The destination buffer for the deserialized string.
 * @param dest_size The size of the destination buffer.
 * @param src The source buffer for the deserialized string.
 * @return The number of bytes read from the source buffer or -1 if the buffer is too small.
 */
int deserialize_string(char* const dest, const size_t dest_size, const uint8_t* const src);

/**
 * Deserialize a buffer from a buffer.
 *
 * @param dest The destination buffer for the deserialized buffer.
 * @param dest_size The size of the destination buffer.
 * @param src The source buffer for the deserialized buffer.
 * @param src_size The size of the source buffer.
 * @return The number of bytes read from the source buffer or -1 if the buffer is too small.
 */
int deserialize_buffer(uint8_t* const dest, const size_t dest_size, const uint8_t* const src);

#endif