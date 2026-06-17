// UART master link to the Servo2040 slave. Encodes outbound command frames and parses
// inbound status/fault frames. Frame format and CRC live in protocol/protocol.h.
#pragma once

#include <Arduino.h>

#include <cstdint>

#include "config.h"
#include "protocol/protocol.h"

namespace koda {

class ServoLink {
 public:
  void begin();

  // Stream the 12 logical servo angles (degrees) to the slave as one SERVO_TARGETS
  // frame. NaN in a slot is sent as the SERVO_HOLD sentinel.
  void send_targets(const float servo_deg[cfg::NUM_SERVOS]);

  void send_mode(proto::SlaveMode mode);
  void send_led(uint8_t idx, uint8_t r, uint8_t g, uint8_t b);
  void ping();

  // Drain the RX buffer, parsing any complete frames. Call once per tick.
  void poll();

  bool    link_ok() const { return link_ok_; }
  uint8_t slave_mode() const { return slave_mode_; }
  uint8_t last_fault() const { return last_fault_; }

 private:
  void write_frame(uint8_t type, const uint8_t* payload, uint8_t len);
  void handle_frame(uint8_t type, const uint8_t* payload, uint8_t len);

  HardwareSerial uart_{cfg::LINK_UART_NUM};

  // RX parser state machine (mirrors PROTOCOL.md).
  enum class RxState { WaitSof, Type, Len, Payload, CrcLo, CrcHi } rx_state_ = RxState::WaitSof;
  uint8_t  rx_type_ = 0;
  uint8_t  rx_len_  = 0;
  uint8_t  rx_idx_  = 0;
  uint8_t  rx_buf_[260];
  uint16_t rx_crc_  = 0;

  bool     link_ok_   = false;
  uint8_t  slave_mode_ = 0;
  uint8_t  last_fault_ = 0;
  uint32_t last_rx_ms_ = 0;
};

}  // namespace koda
