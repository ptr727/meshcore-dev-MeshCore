#include "AutoDiscoverRTCClock.h"
#include "RTClib.h"
#include <Melopero_RV3028.h>
#include "RTC_RX8130CE.h"

static RTC_DS3231 rtc_3231;
static bool ds3231_success = false;

static Melopero_RV3028 rtc_rv3028;
static bool rv3028_success = false;

static RTC_PCF8563 rtc_8563;
static bool rtc_8563_success = false;

static RTC_RX8130CE rtc_8130;
static bool rtc_8130_success = false;

#define DS3231_ADDRESS   0x68
#define RV3028_ADDRESS   0x52
#define PCF8563_ADDRESS  0x51
#define RX8130CE_ADDRESS 0x32

// RV3028 registers used to store its configuration in EEPROM (RV-3028-C7 Application Manual, 4.6)
#define RV3028_STATUS      0x0E  // bit 7: EEbusy
#define RV3028_CONTROL1    0x0F  // bit 3: EERD, disables the automatic refresh from EEPROM
#define RV3028_EE_COMMAND  0x27
#define RV3028_EE_UPDATE   0x11  // copy all configuration RAM to EEPROM
#define RV3028_EE_REFRESH  0x12  // copy all configuration EEPROM to RAM
#define RV3028_EE_CLKOUT   0x35
#define RV3028_EE_BACKUP   0x37

// Configuration MeshCore sets: { register, mask, value }. Bits outside the mask keep the chip's
// own values, notably 37h bit 7, the LSB of the factory frequency calibration.
static const uint8_t rv3028_config[][3] = {
  { RV3028_EE_CLKOUT, 0x80, 0x00 },  // CLKOE = 0: CLKOUT pin off
  // TCE = 1, FEDE = 1 (manual: "should always be set to 1"), BSM = 01 (DSM), TCR = 3 kOhm
  { RV3028_EE_BACKUP, 0x3F, 0x34 },
};
#define RV3028_CONFIG_COUNT (sizeof(rv3028_config) / sizeof(rv3028_config[0]))

// Melopero's readFromRegister() returns 0xFF when the I2C transfer fails, and its writes report
// nothing. A failed read must never be written back, least of all to the EEPROM, so these check.
static bool rv3028Read(uint8_t reg, uint8_t& val) {
  TwoWire* wire = rtc_rv3028.i2c;
  wire->beginTransmission(RV3028_ADDRESS);
  wire->write(reg);
  if (wire->endTransmission() != 0 || wire->requestFrom((uint8_t)RV3028_ADDRESS, (uint8_t)1) != 1) {
    return false;
  }
  val = wire->read();
  return true;
}

static bool rv3028Write(uint8_t reg, uint8_t val) {
  TwoWire* wire = rtc_rv3028.i2c;
  wire->beginTransmission(RV3028_ADDRESS);
  wire->write(reg);
  wire->write(val);
  return wire->endTransmission() == 0;
}

// Waits for EEbusy to clear: ~66 ms after the RTC powers on, ~63 ms for an Update
static bool rv3028EepromIdle() {
  for (int i = 0; i < 20; i++) {
    uint8_t status;
    if (rv3028Read(RV3028_STATUS, status) && (status & 0x80) == 0) return true;
    delay(10);
  }
  return false;
}

// Manual 4.6.7: wait 10 ms after an Update, 1 ms after a Refresh, before checking EEbusy
static bool rv3028EepromCommand(uint8_t cmd, uint32_t wait_ms) {
  if (!rv3028Write(RV3028_EE_COMMAND, 0x00) || !rv3028Write(RV3028_EE_COMMAND, cmd)) return false;
  delay(wait_ms);
  return rv3028EepromIdle();
}

// Stores rv3028_config in the RV3028's EEPROM. Setting only the RAM mirror does not last: while
// EERD = 0 the chip reloads it from EEPROM each day at midnight (manual 4.6.2, 4.6.9), which
// turns the switchover and trickle charger back off. The EEPROM is written only when it does
// not already hold the configuration. Returns false if the configuration was not stored.
static bool rv3028StoreConfig() {
  uint8_t ctrl1;
  if (!rv3028Read(RV3028_CONTROL1, ctrl1)) return false;

  // Manual 4.6.7: set EERD = 1 to disable the automatic refresh, then wait for EEbusy = 0
  bool ok = rv3028Write(RV3028_CONTROL1, ctrl1 | 0x08) && rv3028EepromIdle();

  // Refresh first, so the comparison is against what the EEPROM holds rather than the RAM
  ok = ok && rv3028EepromCommand(RV3028_EE_REFRESH, 1);
  bool changed = false;
  for (size_t i = 0; ok && i < RV3028_CONFIG_COUNT; i++) {
    uint8_t reg = rv3028_config[i][0], mask = rv3028_config[i][1], val = rv3028_config[i][2];
    uint8_t old;
    ok = rv3028Read(reg, old);
    if (ok && (old & mask) != val) {
      ok = rv3028Write(reg, (old & ~mask) | val);
      changed = true;
    }
  }
  if (ok && changed) {
    // Refresh after the Update, so the read-back sees what the EEPROM now holds
    ok = rv3028EepromCommand(RV3028_EE_UPDATE, 10) && rv3028EepromCommand(RV3028_EE_REFRESH, 1);
    for (size_t i = 0; ok && i < RV3028_CONFIG_COUNT; i++) {
      uint8_t now;
      ok = rv3028Read(rv3028_config[i][0], now);
      ok = ok && (now & rv3028_config[i][1]) == rv3028_config[i][2];
    }
  }

  // EERD = 0 on every path once it may have been set
  return rv3028Write(RV3028_CONTROL1, ctrl1 & ~0x08) && ok;
}

bool AutoDiscoverRTCClock::i2c_probe(TwoWire& wire, uint8_t addr) {
  wire.beginTransmission(addr);
  uint8_t error = wire.endTransmission();
  return (error == 0);
}

void AutoDiscoverRTCClock::begin(TwoWire& wire) {
  #if !defined(DISABLE_DS3231_PROBE)
  if (i2c_probe(wire, DS3231_ADDRESS)) {
    ds3231_success = rtc_3231.begin(&wire);
  }
  #endif

  if (i2c_probe(wire, RV3028_ADDRESS)) {
    rtc_rv3028.initI2C(wire);
    // Direct Switching Mode (DSM): when VDD < VBACKUP, switchover occurs from VDD to VBACKUP
    if (!rv3028StoreConfig()) {
      // Fall back to setting the RAM mirror only, which holds until the next refresh
      for (size_t i = 0; i < RV3028_CONFIG_COUNT; i++) {
        uint8_t reg = rv3028_config[i][0], mask = rv3028_config[i][1], val = rv3028_config[i][2];
        uint8_t old;
        if (rv3028Read(reg, old)) rv3028Write(reg, (old & ~mask) | val);
      }
    }
    rtc_rv3028.set24HourMode(); // Set the device to use the 24hour format (default) instead of the 12 hour format
    rv3028_success = true;
  }

  if (i2c_probe(wire, PCF8563_ADDRESS)) {
    MESH_DEBUG_PRINTLN("PCF8563: Found");
    rtc_8563_success = rtc_8563.begin(&wire);
  }

  if (i2c_probe(wire, RX8130CE_ADDRESS)) {
    MESH_DEBUG_PRINTLN("RX8130CE: Found");
    rtc_8130.begin(&wire);
    rtc_8130_success = true;
    MESH_DEBUG_PRINTLN("RX8130CE: Initialized");
  }
}

uint32_t AutoDiscoverRTCClock::getCurrentTime() {
  if (ds3231_success) {
    return rtc_3231.now().unixtime();
  }

  if (rv3028_success) {
    return DateTime(
        rtc_rv3028.getYear(),
        rtc_rv3028.getMonth(),
        rtc_rv3028.getDate(),
        rtc_rv3028.getHour(),
        rtc_rv3028.getMinute(),
        rtc_rv3028.getSecond()
    ).unixtime();
  }

  if (rtc_8563_success) {
    return rtc_8563.now().unixtime();
  }

  if (rtc_8130_success) {
    MESH_DEBUG_PRINTLN("RX8130CE: Reading time");
    return rtc_8130.now().unixtime();
  }

  return _fallback->getCurrentTime();
}

void AutoDiscoverRTCClock::setCurrentTime(uint32_t time) { 
  if (ds3231_success) {
    rtc_3231.adjust(DateTime(time));
  } else if (rv3028_success) {
    auto dt = DateTime(time);
	  uint8_t weekday = (dt.day() + (uint16_t)((2.6 * dt.month()) - 0.2) - (2 * (dt.year() / 100)) + dt.year() + (uint16_t)(dt.year() / 4) + (uint16_t)(dt.year() / 400)) % 7;
    rtc_rv3028.setTime(dt.year(), dt.month(), weekday, dt.day(), dt.hour(), dt.minute(), dt.second());
  } else if (rtc_8563_success) {
    rtc_8563.adjust(DateTime(time));
  } else if (rtc_8130_success) {
    MESH_DEBUG_PRINTLN("RX8130CE: Setting time");
    rtc_8130.adjust(DateTime(time));
  } else {
    _fallback->setCurrentTime(time);
  }
}
