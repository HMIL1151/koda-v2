#include "comms/servo_link.h"

#include <cmath>

#include "math/vec.h"

namespace koda {

namespace {
constexpr uint32_t kStatusTimeoutMs = 500;   // no status this long → link considered down
}

void ServoLink::begin() {
  uart_.begin(cfg::LINK_BAUD, SERIAL_8N1, cfg::LINK_RX_PIN, cfg::LINK_TX_PIN);
}

void ServoLink::write_frame(uint8_t type, const uint8_t* payload, uint8_t len) {
  uint8_t header[3] = {proto::SOF, type, len};

  // CRC covers TYPE, LEN, PAYLOAD (not SOF) — over one contiguous buffer.
  uint8_t crc_src[2 + 255];
  crc_src[0] = type;
  crc_src[1] = len;
  for (uint8_t i = 0; i < len; ++i) crc_src[2 + i] = payload[i];
  const uint16_t crc = proto::crc16(crc_src, 2 + len);

  uart_.write(header, 3);
  if (len) uart_.write(payload, len);
  uint8_t crc_le[2] = {static_cast<uint8_t>(crc & 0xFF),
                       static_cast<uint8_t>((crc >> 8) & 0xFF)};
  uart_.write(crc_le, 2);
}

void ServoLink::send_targets(const float servo_deg[cfg::NUM_SERVOS]) {
  uint8_t payload[2 * cfg::NUM_SERVOS];
  for (int i = 0; i < cfg::NUM_SERVOS; ++i) {
    int16_t v;
    if (std::isnan(servo_deg[i])) {
      v = proto::SERVO_HOLD;
    } else {
      const float centi = roundf(servo_deg[i] * 100.0f);
      v = static_cast<int16_t>(clampf(centi, -32760.0f, 32760.0f));
    }
    payload[2 * i]     = static_cast<uint8_t>(v & 0xFF);
    payload[2 * i + 1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  }
  write_frame(proto::SERVO_TARGETS, payload, sizeof(payload));
}

void ServoLink::send_mode(proto::SlaveMode mode) {
  uint8_t p = static_cast<uint8_t>(mode);
  write_frame(proto::SET_MODE, &p, 1);
}

void ServoLink::send_led(uint8_t idx, uint8_t r, uint8_t g, uint8_t b) {
  uint8_t p[4] = {idx, r, g, b};
  write_frame(proto::LED, p, 4);
}

void ServoLink::ping() { write_frame(proto::PING, nullptr, 0); }

void ServoLink::handle_frame(uint8_t type, const uint8_t* payload, uint8_t len) {
  last_rx_ms_ = millis();
  switch (type) {
    case proto::STATUS:
      if (len >= 1) slave_mode_ = payload[0];
      last_fault_ = 0;
      break;
    case proto::FAULT:
      if (len >= 1) last_fault_ = payload[0];
      break;
    case proto::PONG:
      break;
    default:
      break;
  }
}

void ServoLink::poll() {
  while (uart_.available()) {
    const uint8_t b = static_cast<uint8_t>(uart_.read());
    switch (rx_state_) {
      case RxState::WaitSof:
        if (b == proto::SOF) rx_state_ = RxState::Type;
        break;
      case RxState::Type:
        rx_type_ = b;
        rx_state_ = RxState::Len;
        break;
      case RxState::Len:
        rx_len_ = b;
        rx_idx_ = 0;
        rx_state_ = (rx_len_ == 0) ? RxState::CrcLo : RxState::Payload;
        break;
      case RxState::Payload:
        rx_buf_[rx_idx_++] = b;
        if (rx_idx_ >= rx_len_) rx_state_ = RxState::CrcLo;
        break;
      case RxState::CrcLo:
        rx_crc_ = b;
        rx_state_ = RxState::CrcHi;
        break;
      case RxState::CrcHi: {
        rx_crc_ |= static_cast<uint16_t>(b) << 8;
        // Verify against CRC over TYPE, LEN, PAYLOAD.
        uint8_t src[2 + 255];
        src[0] = rx_type_;
        src[1] = rx_len_;
        for (uint8_t i = 0; i < rx_len_; ++i) src[2 + i] = rx_buf_[i];
        if (proto::crc16(src, 2 + rx_len_) == rx_crc_) {
          handle_frame(rx_type_, rx_buf_, rx_len_);
        }
        rx_state_ = RxState::WaitSof;
        break;
      }
    }
  }
  link_ok_ = (millis() - last_rx_ms_) < kStatusTimeoutMs;
}

}  // namespace koda
