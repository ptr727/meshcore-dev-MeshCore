#pragma once

#include <Mesh.h>
#include <Arduino.h>
#include <Wire.h>

class AutoDiscoverRTCClock : public mesh::RTCClock {
public:
  // Which chip begin() bound, in the order it probes for them. The order is the priority order:
  // where a board somehow answers at two of these addresses, the earlier one keeps time.
  enum Driver : uint8_t {
    DRIVER_NONE = 0,
    DRIVER_DS3231,
    DRIVER_RV3028,
    DRIVER_PCF8563,
    DRIVER_RX8130CE,
  };

private:
  mesh::RTCClock* _fallback;
  Driver _driver = DRIVER_NONE;   // what is keeping time; NONE means the fallback is
  uint8_t _address = 0;           // address the bound chip answered at, 0 when none

  bool i2c_probe(TwoWire& wire, uint8_t addr);
  void bindDriver(Driver driver, uint8_t address);
public:
  AutoDiscoverRTCClock(mesh::RTCClock& fallback) : _fallback(&fallback) { }

  void begin(TwoWire& wire);
  uint32_t getCurrentTime() override;
  void setCurrentTime(uint32_t time) override;

  void tick() override {
    _fallback->tick();   // is typically VolatileRTCClock, which now needs tick()
  }

  // Reporting, for `hwinfo`. With no chip found these defer to the fallback, so the report names
  // the clock that is really keeping time rather than the one this class nominally is. The
  // fallback is per-variant -- VolatileRTCClock on some boards, ESP32RTCClock on others -- so it
  // is asked rather than assumed.
  const char* getDriverName() const override;
  uint8_t getDriverAddress() const override { return _address; }
  bool isFallbackClock() const override { return _driver == DRIVER_NONE; }
};
