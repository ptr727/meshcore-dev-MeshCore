#pragma once

#include <CayenneLPP.h>
#include "sensors/LocationProvider.h"

#define TELEM_PERM_BASE         0x01   // 'base' permission includes battery
#define TELEM_PERM_LOCATION     0x02
#define TELEM_PERM_ENVIRONMENT  0x04   // permission to access environment sensors

#define TELEM_CHANNEL_SELF   1   // LPP data channel for 'self' device

// A device that answered during the boot I2C scan, and the driver that claimed it (if any).
// The bus is part of the identity: the same address on two buses is two different devices.
struct I2CDeviceInfo {
  const char* name;      // driver that claimed the address, or NULL when nothing did
  uint8_t     address;
  uint8_t     bus;       // 0 = Wire, 1 = Wire1
  uint8_t     channel;   // LPP channel if it is a telemetry sensor, 0 otherwise
};

class SensorManager {
public:
  double node_lat, node_lon;  // modify these, if you want to affect Advert location
  double node_altitude;       // altitude in meters

  SensorManager() { node_lat = 0; node_lon = 0; node_altitude = 0; }
  virtual bool begin() { return false; }
  virtual bool querySensors(uint8_t requester_permissions, CayenneLPP& telemetry) { return false; }
  virtual void loop() { }
  virtual int getNumSettings() const { return 0; }
  virtual const char* getSettingName(int i) const { return NULL; }
  virtual const char* getSettingValue(int i) const { return NULL; }
  virtual bool setSettingValue(const char* name, const char* value) { return false; }
  virtual LocationProvider* getLocationProvider() { return NULL; }
  virtual bool isGPSDetected() const { return false; }

  // Hardware inventory, for `hwinfo`. The defaults say "this manager has nothing to report",
  // which is deliberately not the same as "nothing is there": a manager that never scans a bus
  // must not be reported as having scanned one and found it empty. Managers that do not override
  // these -- the plain base class and every custom subclass -- need no changes.
  virtual bool hasHardwareInventory() const { return false; }
  virtual int  getNumDetectedDevices() const { return 0; }
  virtual bool getDetectedDevice(int i, I2CDeviceInfo& out) const { return false; }
  virtual int  getNumActiveSensors() const { return 0; }
  virtual bool getActiveSensor(int i, I2CDeviceInfo& out) const { return false; }

  // Helper functions to manage setting by keys (useful in many places ...)
  const char* getSettingByKey(const char* key) {
    int num = getNumSettings();
    for (int i = 0; i < num; i++) {
      if (strcmp(getSettingName(i), key) == 0) {
        return getSettingValue(i);
      }
    }
    return NULL;
  }
};
