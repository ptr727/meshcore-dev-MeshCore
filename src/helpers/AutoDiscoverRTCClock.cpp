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

// RV-3028-C7 clock registers, contiguous from 0x00
#define RV3028_REG_SECONDS  0x00
#define RV3028_NUM_CLOCK_REGS  7
#define RV3028_REG_YEAR  0x06
#define RV3028_REG_STATUS  0x0E
#define RV3028_STATUS_BSF  0x20  // backup switchover flag

static TwoWire* rv3028_wire = NULL;
static bool rv3028_good = false;   // rv3028_good_time holds an accepted or set time
static uint32_t rv3028_good_time;
static unsigned long rv3028_good_millis;  // millis() at rv3028_good_time
static bool rv3028_hold = false;   // a time write is not confirmed, so the RTC is not read

static inline uint8_t bcd_to_dec(uint8_t bcd) {
  return (uint8_t)((bcd >> 4) * 10 + (bcd & 0x0F));
}

static inline uint8_t dec_to_bcd(uint8_t dec) {
  return (uint8_t)(((dec / 10) << 4) | (dec % 10));
}

// A corrupt register can hold a non-BCD nibble that still decodes into a
// plausible range (0x0A decodes to 10), so check the nibbles themselves.
static inline bool is_bcd(uint8_t b) {
  return (b & 0x0F) <= 9 && (b >> 4) <= 9;
}

static bool rv3028_read_status(uint8_t& status) {
  TwoWire& wire = *rv3028_wire;
  wire.beginTransmission(RV3028_ADDRESS);
  wire.write((uint8_t)RV3028_REG_STATUS);
  if (wire.endTransmission(false) != 0 || wire.requestFrom((uint8_t)RV3028_ADDRESS, (uint8_t)1) != 1) {
    return false;
  }
  status = wire.read();
  return true;
}

static bool rv3028_write_regs(uint8_t reg, const uint8_t* data, uint8_t n) {
  TwoWire& wire = *rv3028_wire;
  wire.beginTransmission(RV3028_ADDRESS);
  wire.write(reg);
  wire.write(data, n);
  return wire.endTransmission() == 0;
}

// Clears BSF if it is set, which works only on VDD (manual 3.7, p. 22). The
// other flags are written back as read.
static bool rv3028_clear_bsf(uint8_t status) {
  if ((status & RV3028_STATUS_BSF) == 0) return true;
  const uint8_t cleared = status & ~RV3028_STATUS_BSF;
  return rv3028_write_regs(RV3028_REG_STATUS, &cleared, 1);
}

// The time runs on from the last accepted or set time. The reference moves
// forward with each call, so millis() wrapping after 49.7 days cannot step it
// back.
static uint32_t rv3028_run_on() {
  const unsigned long secs = (millis() - rv3028_good_millis) / 1000;
  rv3028_good_time += secs;
  rv3028_good_millis += secs * 1000;
  return rv3028_good_time;
}

// Read the RV-3028 clock registers in one burst.
//
// The Melopero getters read one register per transaction, so reading the time
// field by field lets the counters roll over mid-read. Reading most-significant
// first, an hour boundary crossed between the hour and minute reads yields a
// timestamp a full hour in the past (e.g. 15:59:59 -> 16:00:00 reads back as
// 15:00:00); minute and day boundaries misread in the same way.
//
// The fix is the single seven byte read below: those bytes come from one
// uninterrupted transaction, so the counters cannot advance part way through.
// Ending the pointer write without a stop additionally avoids releasing the
// bus, but is not what makes the read coherent, and not every core honours it
// (the STM32 core ignores the flag unless I2C_OTHER_FRAME or USE_HALV2_DRIVER
// is defined). Where it is ignored this degrades to stop-then-start, which is
// what readFromRegister() has always done against this part.
//
// A switchover to VBACKUP during the read garbles it: the chip disables and
// resets its I2C interface (manual 4.2, p. 45; 5.10, p. 95), so the rest of
// the read comes back as 1s, and a byte can still decode to a valid but larger
// value (year 0x26 read as 0x27). So the read is used only if the backup
// switchover flag BSF reads 0 after it. A set BSF is cleared, which is possible
// only on VDD (3.7, p. 22), and the read is rejected. BSF may also be left set
// by an earlier power cut, so the caller reads once more if it was cleared.
//
// Returns 0 with unix_time set, 1 if the read was rejected for a set BSF that
// is now clear, 2 if BSF is set and could not be cleared, or -1 if the
// transfer failed or the fields are not sane.
static int rv3028_read_clock(uint32_t& unix_time) {
  if (rv3028_wire == NULL) return -1;
  TwoWire& wire = *rv3028_wire;

  wire.beginTransmission(RV3028_ADDRESS);
  wire.write((uint8_t)RV3028_REG_SECONDS);
  if (wire.endTransmission(false) != 0) return -1;  // no stop where the core honours it

  if (wire.requestFrom((uint8_t)RV3028_ADDRESS, (uint8_t)RV3028_NUM_CLOCK_REGS)
        != RV3028_NUM_CLOCK_REGS) {
    return -1;
  }

  uint8_t regs[RV3028_NUM_CLOCK_REGS];
  for (uint8_t i = 0; i < RV3028_NUM_CLOCK_REGS; i++) {
    regs[i] = wire.read();
  }

  const uint8_t secs  = regs[0] & 0x7F;
  const uint8_t mins  = regs[1] & 0x7F;
  const uint8_t hours = regs[2] & 0x3F;   // begin() selects 24 hour mode
  // regs[3] is weekday, which DateTime derives itself
  const uint8_t date  = regs[4] & 0x3F;
  const uint8_t month = regs[5] & 0x1F;
  const uint8_t year  = regs[6];

  if (!is_bcd(secs) || !is_bcd(mins) || !is_bcd(hours)
      || !is_bcd(date) || !is_bcd(month) || !is_bcd(year)) {
    return -1;
  }
  // DateTime indexes its month table without checking, so the month is
  // range-checked before isValid() decodes it
  if (bcd_to_dec(month) < 1 || bcd_to_dec(month) > 12) return -1;

  DateTime dt(2000 + bcd_to_dec(year), bcd_to_dec(month), bcd_to_dec(date),
              bcd_to_dec(hours), bcd_to_dec(mins), bcd_to_dec(secs));

  // isValid() round-trips through unixtime(), so it rejects out of range
  // fields and impossible dates such as 31 February in one check.
  if (!dt.isValid()) return -1;

  uint8_t status;
  if (!rv3028_read_status(status)) return -1;
  if (status & RV3028_STATUS_BSF) return rv3028_clear_bsf(status) ? 1 : 2;

  unix_time = dt.unixtime();
  return 0;
}

// Writes the clock registers, bracketed like a read: BSF is cleared first and
// must still read 0 after the write. A switchover or power cut during a write
// leaves the bytes written before it with the rest unchanged, a mixed time
// that reads back as valid. So the year is zeroed first and written last: a
// write cut before the end leaves year 2000, the chip's reset state (manual
// 3.18), which reads as a clock not yet set. Seconds is written first in the
// burst, which resets the prescaler (3.3, p. 14), so no tick lands between the
// writes. Returns true only if the write is confirmed, trying twice.
static bool rv3028_write_time(uint32_t time) {
  DateTime dt(time);
  uint8_t weekday = (dt.day() + (uint16_t)((2.6 * dt.month()) - 0.2) - (2 * (dt.year() / 100)) + dt.year() + (uint16_t)(dt.year() / 4) + (uint16_t)(dt.year() / 400)) % 7;
  const uint8_t regs[RV3028_NUM_CLOCK_REGS] = {
    dec_to_bcd(dt.second()), dec_to_bcd(dt.minute()), dec_to_bcd(dt.hour()), dec_to_bcd(weekday),
    dec_to_bcd(dt.day()), dec_to_bcd(dt.month()), dec_to_bcd(dt.year() - 2000)
  };
  for (int attempt = 0; attempt < 2; attempt++) {
    uint8_t status;
    const uint8_t zero_year = 0;
    if (rv3028_read_status(status) && rv3028_clear_bsf(status)
        && rv3028_write_regs(RV3028_REG_YEAR, &zero_year, 1)
        && rv3028_write_regs(RV3028_REG_SECONDS, regs, RV3028_NUM_CLOCK_REGS - 1)
        && rv3028_write_regs(RV3028_REG_YEAR, &regs[RV3028_NUM_CLOCK_REGS - 1], 1)
        && rv3028_read_status(status) && (status & RV3028_STATUS_BSF) == 0) {
      return true;
    }
  }
  return false;
}

bool AutoDiscoverRTCClock::i2c_probe(TwoWire& wire, uint8_t addr) {
  wire.beginTransmission(addr);
  uint8_t error = wire.endTransmission();
  return (error == 0);
}

void AutoDiscoverRTCClock::begin(TwoWire& wire) {
  #if !defined(DISABLE_DS3231_PROBE)
  if (i2c_probe(wire, DS3231_ADDRESS)) {
    MESH_DEBUG_PRINTLN("DS3231: Found");
    ds3231_success = rtc_3231.begin(&wire);
  }
  #endif

  if (i2c_probe(wire, RV3028_ADDRESS)) {
    MESH_DEBUG_PRINTLN("RV3028: Found");
    rv3028_wire = &wire;
    rtc_rv3028.initI2C(wire);
    rtc_rv3028.writeToRegister(0x35, 0x00);
    rtc_rv3028.writeToRegister(0x37, 0xB4); // Direct Switching Mode (DSM): when VDD < VBACKUP, switchover occurs from VDD to VBACKUP
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

  MESH_DEBUG_PRINTLN("RTC: using %s",
    ds3231_success   ? "DS3231" :
    rv3028_success   ? "RV3028" :
    rtc_8563_success ? "PCF8563" :
    rtc_8130_success ? "RX8130CE" : "none (falling back to volatile clock)");
}

uint32_t AutoDiscoverRTCClock::getCurrentTime() {
  if (ds3231_success) {
    return rtc_3231.now().unixtime();
  }

  if (rv3028_success) {
    int r = -1;
    if (!rv3028_hold) {
      uint32_t unix_time;
      r = rv3028_read_clock(unix_time);
      if (r == 1) r = rv3028_read_clock(unix_time);  // BSF was cleared
      if (r == 0) {
        rv3028_good = true;
        rv3028_good_time = unix_time;
        rv3028_good_millis = millis();
        return unix_time;
      }
    }

    // A rejected read is never used: the time runs on from the last accepted
    // or set one, or, before there is one, comes from the fallback clock.
    // Reaching here means a time write is not yet confirmed, or the read's
    // transfer errored, came up short, its decoded fields failed validation,
    // or BSF was set, not that the core ignored the no-stop flag: where it is
    // ignored, endTransmission() still reports success and the read proceeds.
    // Those causes tend to persist, and getCurrentTime() runs on every
    // received packet, so report it once.
    static bool read_failure_logged = false;
    if (!rv3028_hold && !read_failure_logged) {
      read_failure_logged = true;
      MESH_DEBUG_PRINTLN("RV3028: time read rejected (%d)", r);
    }
    if (!rv3028_good) return _fallback->getCurrentTime();
    const uint32_t now = rv3028_run_on();
    // An unconfirmed write may have left a mixed time in the chip, so the
    // write is repeated until one is confirmed
    if (rv3028_hold && rv3028_write_time(now)) rv3028_hold = false;
    return now;
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
    // Until a write is confirmed the RTC is not read: this boot runs on from
    // the time being set, and getCurrentTime() repeats the write
    rv3028_hold = !rv3028_write_time(time);
    if (rv3028_hold) MESH_DEBUG_PRINTLN("RV3028: time write not confirmed, will retry");
    rv3028_good = true;
    rv3028_good_time = time;
    rv3028_good_millis = millis();
  } else if (rtc_8563_success) {
    rtc_8563.adjust(DateTime(time));
  } else if (rtc_8130_success) {
    MESH_DEBUG_PRINTLN("RX8130CE: Setting time");
    rtc_8130.adjust(DateTime(time));
  } else {
    _fallback->setCurrentTime(time);
  }
}
