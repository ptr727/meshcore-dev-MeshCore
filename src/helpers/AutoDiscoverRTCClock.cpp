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

bool AutoDiscoverRTCClock::i2c_probe(TwoWire& wire, uint8_t addr) {
  wire.beginTransmission(addr);
  uint8_t error = wire.endTransmission();
  return (error == 0);
}

// An ACK only says that something answers at an RTC's address. An IMU at 0x68 or an EEPROM in
// 0x50-0x57 answers too, and would then be read as the clock and written on every time sync.
// So each RTC is adopted only if its seven time registers read like that chip's, per the
// datasheets cited below.

static bool rtcReadTime(TwoWire& wire, uint8_t addr, uint8_t reg, uint8_t t[7]) {
  wire.beginTransmission(addr);
  wire.write(reg);
  if (wire.endTransmission() != 0) return false;
  if (wire.requestFrom(addr, (uint8_t)7) != 7) return false;
  for (int i = 0; i < 7; i++) t[i] = wire.read();
  return true;
}

// True if two reads of the time registers both rule the device out. A failed read proves
// nothing, so the device is adopted as before, and one corrupted read cannot hide a real RTC.
static bool rtcRuledOut(TwoWire& wire, uint8_t addr, uint8_t reg,
                        bool (*ruled_out)(const uint8_t t[7])) {
  for (int i = 0; i < 2; i++) {
    uint8_t t[7];
    if (!rtcReadTime(wire, addr, reg, t) || !ruled_out(t)) return false;
  }
  MESH_DEBUG_PRINTLN("RTC: device at 0x%02X does not read like the RTC expected there", addr);
  return true;
}

static bool anyBitSet(const uint8_t t[7], const uint8_t zero[7]) {
  for (int i = 0; i < 7; i++) {
    if (t[i] & zero[i]) return true;
  }
  return false;
}

// DS3231, 00h-06h: bits shown as 0 in Figure 1 (Maxim 19-5170 Rev 10, p. 11)
static bool ds3231RuledOut(const uint8_t t[7]) {
  static const uint8_t zero[7] = { 0x80, 0x80, 0x80, 0xF8, 0xC0, 0x60, 0x00 };
  return anyBitSet(t, zero);
}

// RV3028, 00h-06h: bits that always read 0 (RV-3028-C7 App Manual Rev 1.4, 3.2-3.3, pp. 12-14)
static bool rv3028RuledOut(const uint8_t t[7]) {
  static const uint8_t zero[7] = { 0x80, 0x80, 0xC0, 0xF8, 0xC0, 0xE0, 0x00 };
  return anyBitSet(t, zero);
}

// RX8130CE, 10h-16h: bits whose "read value is always 0" (Epson ETM50E-10, 13.2.1, p. 22)
static bool rx8130ceRuledOut(const uint8_t t[7]) {
  static const uint8_t zero[7] = { 0x80, 0x80, 0xC0, 0x80, 0xC0, 0xE0, 0x00 };
  return anyBitSet(t, zero);
}

static bool bcdInRange(uint8_t v, uint8_t lo, uint8_t hi) {
  if ((v & 0x0F) > 9 || (v >> 4) > 9) return false;
  uint8_t d = (v >> 4) * 10 + (v & 0x0F);
  return d >= lo && d <= hi;
}

// PCF8563, 02h-08h. Its unused bits are "not relevant" rather than 0, so this checks the field
// ranges instead (NXP PCF8563 Rev 11.1, Table 4, p. 10). With VL set (02h bit 7) the time is
// undefined (Table 27, p. 24), so a chip that lost power is still adopted and can be set.
static bool pcf8563RuledOut(const uint8_t t[7]) {
  bool all_ff = true;
  for (int i = 0; i < 7; i++) all_ff = all_ff && t[i] == 0xFF;
  if (all_ff) return true;  // an erased EEPROM, not a chip whose VL is set
  if (t[0] & 0x80) return false;
  return !(bcdInRange(t[0] & 0x7F, 0, 59) && bcdInRange(t[1] & 0x7F, 0, 59)
           && bcdInRange(t[2] & 0x3F, 0, 23) && bcdInRange(t[3] & 0x3F, 1, 31)
           && (t[4] & 0x07) <= 6 && bcdInRange(t[5] & 0x1F, 1, 12) && bcdInRange(t[6], 0, 99));
}

void AutoDiscoverRTCClock::begin(TwoWire& wire) {
  #if !defined(DISABLE_DS3231_PROBE)
  if (i2c_probe(wire, DS3231_ADDRESS)
      && !rtcRuledOut(wire, DS3231_ADDRESS, 0x00, ds3231RuledOut)) {
    ds3231_success = rtc_3231.begin(&wire);
  }
  #endif

  if (i2c_probe(wire, RV3028_ADDRESS)
      && !rtcRuledOut(wire, RV3028_ADDRESS, 0x00, rv3028RuledOut)) {
    rtc_rv3028.initI2C(wire);
    rtc_rv3028.writeToRegister(0x35, 0x00);
    rtc_rv3028.writeToRegister(0x37, 0xB4); // Direct Switching Mode (DSM): when VDD < VBACKUP, switchover occurs from VDD to VBACKUP
    rtc_rv3028.set24HourMode(); // Set the device to use the 24hour format (default) instead of the 12 hour format
    rv3028_success = true;
  }

  if (i2c_probe(wire, PCF8563_ADDRESS)
      && !rtcRuledOut(wire, PCF8563_ADDRESS, 0x02, pcf8563RuledOut)) {
    MESH_DEBUG_PRINTLN("PCF8563: Found");
    rtc_8563_success = rtc_8563.begin(&wire);
  }

  if (i2c_probe(wire, RX8130CE_ADDRESS)
      && !rtcRuledOut(wire, RX8130CE_ADDRESS, 0x10, rx8130ceRuledOut)) {
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
