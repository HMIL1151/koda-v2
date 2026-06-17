// Analog source abstraction. HallSensors reads raw counts through this interface so it
// doesn't care whether the sensors hang off external ADS1115s, the on-chip ADC, or a
// mock. The concrete ADS1115 implementation lives in adc.cpp.
#pragma once

#include "config.h"

namespace koda {

struct AnalogSource {
  virtual ~AnalogSource() = default;
  virtual bool begin() = 0;                 // returns false if the hardware isn't found
  virtual int  read_raw(int sensor) = 0;    // raw count for global sensor index 0..N-1
  virtual int  max_count() const = 0;       // full-scale count (for reference)
  virtual bool ok() const = 0;              // is the source healthy?
};

// Two ADS1115 ADCs (4 single-ended channels each) on I2C. Sensor index maps as
// (sensor / 4) → chip, (sensor % 4) → channel.
class Ads1115Source : public AnalogSource {
 public:
  bool begin() override;
  int  read_raw(int sensor) override;
  int  max_count() const override { return cfg::HALL_ADC_MAX; }
  bool ok() const override { return ok_; }

 private:
  bool ok_ = false;
};

}  // namespace koda
