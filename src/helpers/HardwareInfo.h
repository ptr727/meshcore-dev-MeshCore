#pragma once

#include <Arduino.h>
#include <string.h>

// Turn a build flag's value into a string literal (two levels so the argument is expanded first).
#define MC_HW_STR_(x) #x
#define MC_HW_STR(x)  MC_HW_STR_(x)

/**
 * Compile-time hardware identity.
 *
 * MeshCore has no device tree: what a board is made of lives in build flags, in the variant's
 * target.cpp object graph, and in runtime probes whose results are discarded. This class answers
 * the part that is knowable from build flags alone, so that `hwinfo` can report it on a release
 * build. Runtime discovery (RTC, I2C, GNSS) is reported by the objects that do the discovering.
 *
 * Everything here returns a `const char*` into flash or a plain integer. There is no state, no
 * allocation and no copying, so the cost is the string literals themselves.
 *
 * A value that is not knowable is reported as "unknown", and one that is knowably absent as
 * "none". The two are never conflated: "unknown" means nothing looked, "none" means something
 * looked and found nothing.
 */


class HardwareInfo {
public:
  // --- MCU ---------------------------------------------------------------------------------

  /** Broad MCU family, from the platform macro every build already sets. */
  static const char* getMCUFamily() {
  #if defined(NRF52_PLATFORM)
    return "nRF52";
  #elif defined(ESP32_PLATFORM)
    return "ESP32";
  #elif defined(RP2040_PLATFORM)
    return "RP2040";
  #elif defined(STM32_PLATFORM)
    return "STM32";
  #else
    return "unknown";
  #endif
  }

  /**
   * Specific MCU part where a toolchain macro states it outright, otherwise the family.
   * Only macros the toolchain defines itself are consulted -- the part is never inferred from a
   * board name, because a wrong-but-plausible part number is worse than no part number.
   */
  static const char* getMCUChip() {
  #if defined(NRF52_PLATFORM)
    #if defined(NRF52840_XXAA)
      return "nRF52840";
    #elif defined(NRF52833_XXAA)
      return "nRF52833";
    #elif defined(NRF52832_XXAA)
      return "nRF52832";
    #else
      return "nRF52";
    #endif
  #elif defined(ESP32_PLATFORM)
    #if defined(CONFIG_IDF_TARGET_ESP32S3)
      return "ESP32-S3";
    #elif defined(CONFIG_IDF_TARGET_ESP32S2)
      return "ESP32-S2";
    #elif defined(CONFIG_IDF_TARGET_ESP32C6)
      return "ESP32-C6";
    #elif defined(CONFIG_IDF_TARGET_ESP32C3)
      return "ESP32-C3";
    #elif defined(CONFIG_IDF_TARGET_ESP32)
      return "ESP32";
    #else
      return "ESP32";
    #endif
  #elif defined(RP2040_PLATFORM)
    #if defined(PICO_RP2350)
      return "RP2350";
    #elif defined(PICO_RP2040)
      return "RP2040";
    #else
      return "RP2040";
    #endif
  #elif defined(STM32_PLATFORM)
    #if defined(STM32WLxx)
      return "STM32WL";
    #else
      return "STM32";
    #endif
  #else
    return "unknown";
  #endif
  }

  // --- Radio -------------------------------------------------------------------------------

  /**
   * LoRa transceiver part, derived from the RADIO_CLASS build flag.
   *
   * The flag names a driver class (CustomSX1262), not a part, so the "Custom" prefix is dropped
   * to leave the part (SX1262). Deriving it from the flag rather than from a fixed table means a
   * driver added later reports correctly with no change here; a class not following the
   * convention degrades to its own name, which still identifies the radio.
   */
  static const char* getRadioChip() {
  #ifdef RADIO_CLASS
    const char* name = MC_HW_STR(RADIO_CLASS);
    return (strncmp(name, "Custom", 6) == 0) ? name + 6 : name;
  #else
    return "unknown";
  #endif
  }

  // --- Display -----------------------------------------------------------------------------

  /**
   * Display driver class, or "none" when the build has no display.
   *
   * Reported as the driver class name rather than a panel part: a driver covers a family of
   * panels, so naming a part here would be a guess. NullDisplayDriver is the build's own way of
   * saying there is no display, so it reports as "none".
   */
  static const char* getDisplayName() {
  #ifdef DISPLAY_CLASS
    const char* name = MC_HW_STR(DISPLAY_CLASS);
    return (strcmp(name, "NullDisplayDriver") == 0) ? "none" : name;
  #else
    return "none";
  #endif
  }

  static bool hasDisplay() {
    return strcmp(getDisplayName(), "none") != 0;
  }

  // --- Board identity ----------------------------------------------------------------------

  /**
   * Variant slug (the PlatformIO env's variant, e.g. "rak4631").
   * Defined centrally at build time; "unknown" until that lands, never guessed from other flags.
   */
  static const char* getVariantSlug() {
  #ifdef MC_VARIANT_SLUG
    return MC_VARIANT_SLUG;
  #else
    return "unknown";
  #endif
  }

  // --- I2C ---------------------------------------------------------------------------------

  /** Pins for the primary I2C bus (Wire), or -1 when the build does not name them. */
  static int getI2CSdaPin() {
  #ifdef PIN_BOARD_SDA
    return PIN_BOARD_SDA;
  #else
    return -1;
  #endif
  }

  static int getI2CSclPin() {
  #ifdef PIN_BOARD_SCL
    return PIN_BOARD_SCL;
  #else
    return -1;
  #endif
  }

  /**
   * Pins for the secondary I2C bus (Wire1), or -1 when there is no second bus.
   * EnvironmentSensorManager puts sensors on Wire1 exactly when both of these are set, so these
   * two macros are what decide which bus is actually scanned.
   */
  static int getI2C1SdaPin() {
  #if ENV_PIN_SDA && ENV_PIN_SCL
    return ENV_PIN_SDA;
  #else
    return -1;
  #endif
  }

  static int getI2C1SclPin() {
  #if ENV_PIN_SDA && ENV_PIN_SCL
    return ENV_PIN_SCL;
  #else
    return -1;
  #endif
  }

  /** True when sensors live on Wire1 rather than Wire -- mirrors TELEM_WIRE's own condition. */
  static bool sensorsOnSecondaryBus() {
  #if ENV_PIN_SDA && ENV_PIN_SCL
    return true;
  #else
    return false;
  #endif
  }

  // --- GNSS (compile-time wiring only; state is reported by the sensor manager) --------------

  /** True when GNSS support was compiled in at all. */
  static bool isGPSCompiledIn() {
  #if ENV_INCLUDE_GPS
    return true;
  #else
    return false;
  #endif
  }

  static int getGPSRxPin() {
  #ifdef PIN_GPS_RX
    return PIN_GPS_RX;
  #else
    return -1;
  #endif
  }

  static int getGPSTxPin() {
  #ifdef PIN_GPS_TX
    return PIN_GPS_TX;
  #else
    return -1;
  #endif
  }

  static uint32_t getGPSBaudRate() {
  #ifdef GPS_BAUD_RATE
    return GPS_BAUD_RATE;
  #else
    return 9600;   // MicroNMEALocationProvider's own default when the flag is absent
  #endif
  }

  // --- Links -------------------------------------------------------------------------------

  /** Packet bridge transport, or "none". */
  static const char* getBridgeType() {
  #if defined(WITH_RS232_BRIDGE)
    return "rs232";
  #elif defined(WITH_ESPNOW_BRIDGE)
    return "espnow";
  #else
    return "none";
  #endif
  }

  static bool hasEthernet() {
  #if defined(ETHERNET_ENABLED)
    return true;
  #else
    return false;
  #endif
  }
};
