#include "sensors/adc.h"

#include <Adafruit_ADS1X15.h>
#include <Wire.h>

namespace koda {

namespace {
Adafruit_ADS1115 g_ads[2];
bool g_present[2] = {false, false};
}  // namespace

bool Ads1115Source::begin() {
  Wire.begin(cfg::HALL_I2C_SDA, cfg::HALL_I2C_SCL);

  ok_ = true;
  for (int i = 0; i < 2; ++i) {
    g_present[i] = g_ads[i].begin(cfg::ADS1115_ADDR[i], &Wire);
    if (g_present[i]) {
      // ±6.144 V range comfortably covers a 3.3 V ratiometric sensor; 860 SPS keeps a
      // single conversion near ~1.2 ms so a round-robin read doesn't stall the loop.
      g_ads[i].setGain(GAIN_TWOTHIRDS);
      g_ads[i].setDataRate(RATE_ADS1115_860SPS);
    } else {
      ok_ = false;   // a missing ADC makes the source unhealthy, but we keep running
    }
  }
  return ok_;
}

int Ads1115Source::read_raw(int sensor) {
  const int chip = sensor / 4;
  const int chan = sensor % 4;
  if (chip < 0 || chip >= 2 || !g_present[chip]) return 0;

  const int16_t v = g_ads[chip].readADC_SingleEnded(chan);
  return v < 0 ? 0 : v;     // single-ended: clamp tiny negatives from noise
}

}  // namespace koda
