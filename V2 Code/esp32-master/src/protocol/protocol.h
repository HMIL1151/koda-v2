// Shared UART wire format for the ESP32 ⇄ Servo2040 link. This is the C++ half; the
// MicroPython half is servo2040-slave/protocol.py and MUST stay byte-for-byte in step
// with it. See PROTOCOL.md for the full spec.
#pragma once

#include <cstddef>
#include <cstdint>

namespace proto {

constexpr uint8_t PROTOCOL_VERSION = 1;
constexpr uint8_t SOF = 0xA5;            // start-of-frame sentinel

// Message types. High bit set = slave → master.
enum MsgType : uint8_t {
  SERVO_TARGETS = 0x01,   // 12 × int16 centi-degrees
  SET_MODE      = 0x02,   // 1 byte SlaveMode
  LED           = 0x03,   // idx, r, g, b
  PING          = 0x04,   // heartbeat
  STATUS        = 0x81,
  FAULT         = 0x82,
  PONG          = 0x83,
};

enum SlaveMode : uint8_t { RELAX = 0, ACTIVE = 1, HOLD = 2, SAFE = 3 };

enum FaultCode : uint8_t {
  FAULT_LINK_TIMEOUT  = 1,
  FAULT_MALFORMED     = 2,
  FAULT_OUT_OF_RANGE  = 3,
  FAULT_DRIVER        = 4,
};

// "Hold this channel / don't drive" sentinel for a SERVO_TARGETS slot.
constexpr int16_t SERVO_HOLD = INT16_MIN;

// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over TYPE..PAYLOAD. Identical maths on
// both boards so the slave can verify exactly what we computed.
inline uint16_t crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; ++b)
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
  }
  return crc;
}

}  // namespace proto
