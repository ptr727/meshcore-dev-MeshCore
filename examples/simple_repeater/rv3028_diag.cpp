// THROWAWAY: RV3028 diagnostics for hardware-testing the EEPROM store (fork issue #10, PR #11).
// Never for upstream. Built as a local PlatformIO env extending [rak4631] with the
// RAK_4631_repeater flags plus LORA_TX_POWER=1, LORA_FREQ=927.875, LORA_BW=62.5, LORA_SF=7,
// LORA_CR=5, and build_src_filter + <../examples/simple_repeater>. Serial CLI commands:
//   rv dump                         RAM 0Eh/0Fh/35h/37h, EEPROM 35h/37h, time, store result
//   rv factory                      EEPROM 35h = C0h, 37h = 10h | EEOffset[0] kept; then Refresh
//   rv clr                          clear the status flags (PORF, BSF, ...)
//   rv set YY MM DD hh mm ss        write the RTC time registers directly (decimal, YY = 0-99)
#include <Arduino.h>
#include <Wire.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RVA 0x52

int rv3028_diag_store = -2;  // -2: build has no store; -1 not run; 0 failed; 1 no change; 2 written

static bool rd(uint8_t reg, uint8_t& v) {
  Wire.beginTransmission(RVA);
  Wire.write(reg);
  if (Wire.endTransmission() != 0 || Wire.requestFrom((uint8_t)RVA, (uint8_t)1) != 1) return false;
  v = Wire.read();
  return true;
}

static bool wr(uint8_t reg, uint8_t v) {
  Wire.beginTransmission(RVA);
  Wire.write(reg);
  Wire.write(v);
  return Wire.endTransmission() == 0;
}

static bool idle() {
  for (int i = 0; i < 30; i++) {
    uint8_t st;
    if (rd(0x0E, st) && (st & 0x80) == 0) return true;
    delay(10);
  }
  return false;
}

static bool eeRead(uint8_t addr, uint8_t& v) {
  if (!idle() || !wr(0x25, addr) || !wr(0x27, 0x00) || !wr(0x27, 0x22)) return false;
  delay(2);
  return idle() && rd(0x26, v);
}

static bool eeWrite(uint8_t addr, uint8_t v) {
  if (!idle() || !wr(0x25, addr) || !wr(0x26, v) || !wr(0x27, 0x00) || !wr(0x27, 0x21)) return false;
  delay(20);
  return idle();
}

static uint8_t bcd(int d) { return (uint8_t)(((d / 10) << 4) | (d % 10)); }

bool rv3028_diag(const char* cmd, char* reply) {
  if (strncmp(cmd, "rv ", 3) != 0) return false;
  const char* a = cmd + 3;
  uint8_t c1;
  if (!rd(0x0F, c1)) { strcpy(reply, "rv: i2c read failed"); return true; }

  if (strcmp(a, "dump") == 0) {
    uint8_t st = 0, r35 = 0, r37 = 0, e35 = 0, e37 = 0, t[7] = {0};
    bool ok = rd(0x0E, st) && rd(0x35, r35) && rd(0x37, r37);
    for (int i = 0; ok && i < 7; i++) ok = rd(i, t[i]);
    bool eok = wr(0x0F, c1 | 0x08) && eeRead(0x35, e35) && eeRead(0x37, e37);
    wr(0x0F, c1);  // restore EERD as it was
    sprintf(reply, "%s st=%02X c1=%02X ram35=%02X ram37=%02X ee35=%02X ee37=%02X%s 20%02X-%02X-%02X %02X:%02X:%02X store=%d",
            ok ? "ok" : "RDFAIL", st, c1, r35, r37, e35, e37, eok ? "" : "(EEFAIL)",
            t[6], t[5], t[4], t[2], t[1], t[0], rv3028_diag_store);
    return true;
  }

  if (strcmp(a, "factory") == 0) {
    uint8_t e37 = 0, n35 = 0, n37 = 0;
    bool ok = wr(0x0F, c1 | 0x08) && eeRead(0x37, e37)
              && eeWrite(0x37, (e37 & 0x80) | 0x10) && eeWrite(0x35, 0xC0)
              && wr(0x27, 0x00) && wr(0x27, 0x12);  // Refresh, so RAM matches
    delay(2);
    ok = ok && idle() && eeRead(0x35, n35) && eeRead(0x37, n37);
    wr(0x0F, c1 & ~0x08);  // EERD = 0
    sprintf(reply, "factory %s: ee37 was %02X, now ee35=%02X ee37=%02X", ok ? "ok" : "FAILED", e37, n35, n37);
    return true;
  }

  if (strcmp(a, "clr") == 0) {  // clear PORF, BSF and the other status flags
    sprintf(reply, "clr %s", wr(0x0E, 0x00) ? "ok" : "FAILED");
    return true;
  }

  int y, mo, d, h, mi, s;
  if (sscanf(a, "set %d %d %d %d %d %d", &y, &mo, &d, &h, &mi, &s) == 6) {
    uint8_t t[8] = { 0x00, bcd(s), bcd(mi), bcd(h), 0x00, bcd(d), bcd(mo), bcd(y % 100) };
    Wire.beginTransmission(RVA);
    Wire.write(t, sizeof(t));  // register pointer 00h, then 00h-06h
    sprintf(reply, "set %s", Wire.endTransmission() == 0 ? "ok" : "FAILED");
    return true;
  }

  strcpy(reply, "rv: dump | factory | clr | set YY MM DD hh mm ss");
  return true;
}
