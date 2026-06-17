#include "sensors/hall_store.h"

#include <ArduinoJson.h>
#include <LittleFS.h>

#include "config.h"

namespace koda {
namespace hall_store {

namespace {
bool mount() {
  // `true` formats on first use if the partition is empty.
  return LittleFS.begin(true);
}
}  // namespace

bool load(HallSensors& hall) {
  if (!mount()) return false;
  File f = LittleFS.open(cfg::HALL_CAL_PATH, "r");
  if (!f) return false;

  JsonDocument doc;
  const DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) return false;

  if (static_cast<int>(doc["version"] | 0) != cfg::HALL_CAL_VERSION) return false;
  JsonArray sensors = doc["sensors"].as<JsonArray>();
  if (sensors.isNull() || static_cast<int>(sensors.size()) != cfg::NUM_HALL_SENSORS)
    return false;

  for (int i = 0; i < cfg::NUM_HALL_SENSORS; ++i) {
    HallCal& c = hall.cal(i);
    c.s0 = sensors[i]["s0"] | 0.0f;
    c.s1 = sensors[i]["s1"] | 0.0f;
    c.K = sensors[i]["K"] | 1.0f;
    c.calibrated = sensors[i]["cal"] | false;
  }
  return true;
}

bool save(const HallSensors& hall) {
  if (!mount()) return false;

  JsonDocument doc;
  doc["version"] = cfg::HALL_CAL_VERSION;
  JsonArray sensors = doc["sensors"].to<JsonArray>();
  for (int i = 0; i < cfg::NUM_HALL_SENSORS; ++i) {
    const HallCal& c = hall.cal(i);
    JsonObject o = sensors.add<JsonObject>();
    o["s0"] = c.s0;
    o["s1"] = c.s1;
    o["K"] = c.K;
    o["cal"] = c.calibrated;
  }

  File f = LittleFS.open(cfg::HALL_CAL_PATH, "w");
  if (!f) return false;
  const bool ok = serializeJsonPretty(doc, f) > 0;
  f.close();
  return ok;
}

}  // namespace hall_store
}  // namespace koda
