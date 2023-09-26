/**
 * @file serialize.cpp
 * @author Zach Champion (zachchampion79@gmail.com)
 * 
 * @version 2.0
 * @date 2023-09-19
 */

#include "ar3_hardware_interface/serialize.h"
#include <cstring>

int serialize_int8(uint8_t* const dest, const size_t dest_size, const int8_t val)
{
  if (dest_size < sizeof(int8_t)) return -1;  // not enough space
  *dest = val;
  return sizeof(int8_t);
}

int serialize_int16(uint8_t* const dest, const size_t dest_size, const int16_t val)
{
  if (dest_size < sizeof(int16_t)) return -1;  // not enough space
  dest[0] = val & 0xFF;
  dest[1] = (val >> 8) & 0xFF;
  return sizeof(int16_t);
}

int serialize_int32(uint8_t* const dest, const size_t dest_size, const int32_t val)
{
  if (dest_size < sizeof(int32_t)) return -1;  // not enough space
  dest[0] = val & 0xFF;
  dest[1] = (val >> 8) & 0xFF;
  dest[2] = (val >> 16) & 0xFF;
  dest[3] = (val >> 24) & 0xFF;
  return sizeof(int32_t);
}

int serialize_int64(uint8_t* const dest, const size_t dest_size, const int64_t val)
{
  if (dest_size < sizeof(int64_t)) return -1;  // not enough space
  dest[0] = val & 0xFF;
  dest[1] = (val >> 8) & 0xFF;
  dest[2] = (val >> 16) & 0xFF;
  dest[3] = (val >> 24) & 0xFF;
  dest[4] = (val >> 32) & 0xFF;
  dest[5] = (val >> 40) & 0xFF;
  dest[6] = (val >> 48) & 0xFF;
  dest[7] = (val >> 56) & 0xFF;
  return sizeof(int64_t);
}

int serialize_uint8(uint8_t* const dest, const size_t dest_size, const uint8_t val)
{
  return serialize_int8(dest, dest_size, (int8_t)val);  // same as int8_t
}

int serialize_uint16(uint8_t* const dest, const size_t dest_size, const uint16_t val)
{
  return serialize_int16(dest, dest_size, (int16_t)val);  // same as int16_t
}

int serialize_uint32(uint8_t* const dest, const size_t dest_size, const uint32_t val)
{
  return serialize_int32(dest, dest_size, (int32_t)val);  // same as int32_t
}

int serialize_uint64(uint8_t* const dest, const size_t dest_size, const uint64_t val)
{
  return serialize_int64(dest, dest_size, (int64_t)val);  // same as int64_t
}

int serialize_float(uint8_t* const dest, const size_t dest_size, const float val)
{
  return serialize_int32(dest, dest_size, (int32_t)val);  // same as int32_t
}

int serialize_double(uint8_t* const dest, const size_t dest_size, const double val)
{
  return serialize_int64(dest, dest_size, (int64_t)val);  // same as int64_t
}

int serialize_string(uint8_t* const dest, const size_t dest_size, const char* const str)
{
  const size_t len = strlen(str);                         // get length of string
  if (dest_size < sizeof(uint32_t) + len) return -1;      // not enough space
  const int ret = serialize_int32(dest, dest_size, len);  // serialize length
  if (ret < 0) return ret;                                // error
  memcpy(dest + ret, str, len);                           // copy string
  return ret + len;  // return length of string + length of length
}

int serialize_buffer(uint8_t* const dest, const size_t dest_size, const uint8_t* const buf,
                     const size_t buf_size)
{
  if (dest_size < sizeof(uint32_t) + buf_size) return -1;      // not enough space
  const int ret = serialize_int32(dest, dest_size, buf_size);  // serialize length
  if (ret < 0) return ret;                                     // error
  memcpy(dest + ret, buf, buf_size);                           // copy buffer
  return ret + buf_size;  // return length of buffer + length of length
}

int deserialize_int8(int8_t* const dest, const uint8_t* const src)
{
  *dest = (int8_t)src[0];
  return sizeof(int8_t);
}

int deserialize_int16(int16_t* const dest, const uint8_t* const src)
{
  *dest = (int16_t)(src[0] | (src[1] << 8));
  return sizeof(int16_t);
}

int deserialize_int32(int32_t* const dest, const uint8_t* const src)
{
  *dest = (int32_t)(src[0] | (src[1] << 8) | (src[2] << 16) | (src[3] << 24));
  return sizeof(int32_t);
}

int deserialize_int64(int64_t* const dest, const uint8_t* const src)
{
  int64_t ret = 0;
  for (size_t i = 0; i < sizeof(int64_t); ++i) {
    ret |= (int64_t)src[i] << (i * 8);
  }
  *dest = ret;
  return sizeof(int64_t);
}

int deserialize_uint8(uint8_t* const dest, const uint8_t* const src)
{
  return deserialize_int8((int8_t*)dest, src);  // same as int8_t
}

int deserialize_uint16(uint16_t* const dest, const uint8_t* const src)
{
  return deserialize_int16((int16_t*)dest, src);  // same as int16_t
}

int deserialize_uint32(uint32_t* const dest, const uint8_t* const src)
{
  return deserialize_int32((int32_t*)dest, src);  // same as int32_t
}

int deserialize_uint64(uint64_t* const dest, const uint8_t* const src)
{
  return deserialize_int64((int64_t*)dest, src);  // same as int64_t
}

int deserialize_float(float* const dest, const uint8_t* const src)
{
  return deserialize_int32((int32_t*)dest, src);  // same as int32_t
}

int deserialize_double(double* const dest, const uint8_t* const src)
{
  return deserialize_int64((int64_t*)dest, src);  // same as int64_t
}

int deserialize_string(char* const dest, const size_t dest_size, const uint8_t* const src)
{
  uint32_t len;
  const int ret = deserialize_uint32(&len, src);
  if (ret < 0) return ret;
  if (dest_size < len + 1) return -1;  // not enough space
  memcpy(dest, src + ret, len);        // copy string
  dest[len] = '\0';                    // terminate string
  return ret + len;                    // return size of length + size of string
}

int deserialize_buffer(uint8_t* const dest, const size_t dest_size, const uint8_t* const src)
{
  uint32_t len;
  const int ret = deserialize_uint32(&len, src);
  if (ret < 0) return ret;
  if (dest_size < len) return -1;  // not enough space
  memcpy(dest, src + ret, len);    // copy buffer
  return len;                      // return size of length + size of buffer
}