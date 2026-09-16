#include "AutoDiscoverRTCClock.h"
#include "RTClib.h"
#include <Melopero_RV3028.h>
#include "RTC_RX8130CE.h"

static RTC_DS3231 rtc_3231;
static Melopero_RV3028 rtc_rv3028;
static RTC_PCF8563 rtc_8563;
static RTC_RX8130CE rtc_8130;

#define DS3231_ADDRESS   0x68
#define RV3028_ADDRESS   0x52
#define PCF8563_ADDRESS  0x51
#define RX8130CE_ADDRESS 0x32

bool AutoDiscoverRTCClock::i2c_probe(TwoWire& wire, uint8_t addr) {
  wire.beginTransmission(addr);
  uint8_t error = wire.endTransmission();
  return (error == 0);
}

// Records the first chip to answer, in probe order. Every probe still runs and every chip that
// answers is still initialised exactly as before -- this only remembers which one ends up keeping
// time, which previously could not be asked from outside this file.
void AutoDiscoverRTCClock::bindDriver(Driver driver, uint8_t address) {
  if (_driver == DRIVER_NONE) {
    _driver = driver;
    _address = address;
  }
}

void AutoDiscoverRTCClock::begin(TwoWire& wire) {
  #if !defined(DISABLE_DS3231_PROBE)
  if (i2c_probe(wire, DS3231_ADDRESS)) {
    if (rtc_3231.begin(&wire)) bindDriver(DRIVER_DS3231, DS3231_ADDRESS);
  }
  #endif

  if (i2c_probe(wire, RV3028_ADDRESS)) {
    rtc_rv3028.initI2C(wire);
    rtc_rv3028.writeToRegister(0x35, 0x00);
    rtc_rv3028.writeToRegister(0x37, 0xB4); // Direct Switching Mode (DSM): when VDD < VBACKUP, switchover occurs from VDD to VBACKUP
    rtc_rv3028.set24HourMode(); // Set the device to use the 24hour format (default) instead of the 12 hour format
    bindDriver(DRIVER_RV3028, RV3028_ADDRESS);
  }

  if (i2c_probe(wire, PCF8563_ADDRESS)) {
    MESH_DEBUG_PRINTLN("PCF8563: Found");
    if (rtc_8563.begin(&wire)) bindDriver(DRIVER_PCF8563, PCF8563_ADDRESS);
  }

  if (i2c_probe(wire, RX8130CE_ADDRESS)) {
    MESH_DEBUG_PRINTLN("RX8130CE: Found");
    rtc_8130.begin(&wire);
    bindDriver(DRIVER_RX8130CE, RX8130CE_ADDRESS);
    MESH_DEBUG_PRINTLN("RX8130CE: Initialized");
  }
}

const char* AutoDiscoverRTCClock::getDriverName() const {
  switch (_driver) {
    case DRIVER_DS3231:   return "DS3231";
    case DRIVER_RV3028:   return "RV3028";
    case DRIVER_PCF8563:  return "PCF8563";
    case DRIVER_RX8130CE: return "RX8130CE";
    default:              return _fallback->getDriverName();   // nothing found; name what is
  }
}

uint32_t AutoDiscoverRTCClock::getCurrentTime() {
  switch (_driver) {
    case DRIVER_DS3231:
      return rtc_3231.now().unixtime();

    case DRIVER_RV3028:
      return DateTime(
          rtc_rv3028.getYear(),
          rtc_rv3028.getMonth(),
          rtc_rv3028.getDate(),
          rtc_rv3028.getHour(),
          rtc_rv3028.getMinute(),
          rtc_rv3028.getSecond()
      ).unixtime();

    case DRIVER_PCF8563:
      return rtc_8563.now().unixtime();

    case DRIVER_RX8130CE:
      MESH_DEBUG_PRINTLN("RX8130CE: Reading time");
      return rtc_8130.now().unixtime();

    default:
      return _fallback->getCurrentTime();
  }
}

void AutoDiscoverRTCClock::setCurrentTime(uint32_t time) {
  switch (_driver) {
    case DRIVER_DS3231:
      rtc_3231.adjust(DateTime(time));
      break;

    case DRIVER_RV3028: {
      auto dt = DateTime(time);
      uint8_t weekday = (dt.day() + (uint16_t)((2.6 * dt.month()) - 0.2) - (2 * (dt.year() / 100)) + dt.year() + (uint16_t)(dt.year() / 4) + (uint16_t)(dt.year() / 400)) % 7;
      rtc_rv3028.setTime(dt.year(), dt.month(), weekday, dt.day(), dt.hour(), dt.minute(), dt.second());
      break;
    }

    case DRIVER_PCF8563:
      rtc_8563.adjust(DateTime(time));
      break;

    case DRIVER_RX8130CE:
      MESH_DEBUG_PRINTLN("RX8130CE: Setting time");
      rtc_8130.adjust(DateTime(time));
      break;

    default:
      _fallback->setCurrentTime(time);
      break;
  }
}
