// THROWAWAY: read-only diagnostics for the RTC identity check (fork issue #13). Never for upstream.
//   rv scan          list I2C addresses that ACK (address-only, nothing written to any register)
//   rv id <hexaddr>  run all four RTC identity checks against one address (register reads only)
//   rv adopted       which RTCs AutoDiscoverRTCClock::begin() adopted
#include <Arduino.h>
#include <Wire.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern int rtc_diag_check(TwoWire& wire, uint8_t addr, int chip);  // 1 ruled out, 0 not, -1 read failed
extern void rtc_diag_adopted(char* out);

bool rtc_probe_diag(const char* cmd, char* reply) {
  if (strcmp(cmd, "rv scan") == 0) {
    char* p = reply;
    p += sprintf(p, "ack:");
    for (uint8_t a = 0x08; a < 0x78; a++) {
      Wire.beginTransmission(a);
      if (Wire.endTransmission() == 0 && p - reply < 150) p += sprintf(p, " %02X", a);
    }
    return true;
  }
  if (strncmp(cmd, "rv id ", 6) == 0) {
    uint8_t a = (uint8_t)strtoul(cmd + 6, NULL, 16);
    static const char* names[4] = { "ds3231", "rv3028", "pcf8563", "rx8130ce" };
    char* p = reply;
    p += sprintf(p, "%02X:", a);
    for (int c = 0; c < 4; c++) {
      int r = rtc_diag_check(Wire, a, c);
      p += sprintf(p, " %s=%s", names[c], r == 1 ? "OUT" : r == 0 ? "ok" : "readfail");
    }
    return true;
  }
  unsigned int ra, rr;
  if (sscanf(cmd, "rv raw %x %x", &ra, &rr) == 2) {  // 8 bytes from register rr (read only)
    Wire.beginTransmission((uint8_t)ra);
    Wire.write((uint8_t)rr);
    if (Wire.endTransmission() != 0 || Wire.requestFrom((uint8_t)ra, (uint8_t)8) != 8) {
      strcpy(reply, "raw: read failed");
      return true;
    }
    char* p = reply;
    p += sprintf(p, "%02X@%02X:", ra, rr);
    for (int i = 0; i < 8; i++) p += sprintf(p, " %02X", Wire.read());
    return true;
  }
  if (strcmp(cmd, "rv adopted") == 0) {
    rtc_diag_adopted(reply);
    return true;
  }
  return false;
}
