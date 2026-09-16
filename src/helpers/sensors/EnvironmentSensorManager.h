#pragma once

#include <Mesh.h>
#include <helpers/SensorManager.h>
#include <helpers/sensors/LocationProvider.h>

class EnvironmentSensorManager : public SensorManager {
protected:
  static const int MAX_ACTIVE_SENSORS = 16;

  // Query function pointer + sub-channel index (for multi-channel sensors like INA3221).
  // Sub-channel is 0 for all single-output sensors.
  struct ActiveSensor {
    void    (*query)(uint8_t channel, uint8_t sub_channel, CayenneLPP& telemetry);
    uint8_t   sub_channel;
    uint8_t   table_index;   // into SENSOR_TABLE, so the name and address can be reported
  };

  ActiveSensor _active_sensors[MAX_ACTIVE_SENSORS];
  int          _active_sensor_count = 0;
  uint8_t      next_available_channel = TELEM_CHANNEL_SELF + 1;

  // Retained boot-scan result, so `hwinfo` can answer at any time rather than only in the boot
  // scrollback. One bit per address, snapshotted before the driver walk begins -- that walk
  // clears bits as drivers claim them, so a bitmap kept afterwards would report a claimed device
  // as absent. 16 bytes for the bus, plus one to record which bus it was.
  uint8_t      _i2c_found[16] = {};
  uint8_t      _i2c_bus = 0;
  bool         _i2c_scanned = false;

  bool     gps_detected = false;
  bool     gps_active = false;
  uint32_t gps_update_interval_sec = 1;

  #if ENV_INCLUDE_GPS
  LocationProvider* _location;
  void start_gps();
  void stop_gps();
  void initBasicGPS();
  #ifdef RAK_BOARD
  void rakGPSInit();
  bool gpsIsAwake(uint8_t ioPin);
  #endif
  #endif

public:
  #if ENV_INCLUDE_GPS
  EnvironmentSensorManager(LocationProvider &location): _location(&location){};
  LocationProvider* getLocationProvider() { return _location; }
  bool isGPSDetected() const override { return gps_detected; }
  #else
  EnvironmentSensorManager(){};
  #endif
  bool begin() override;
  bool querySensors(uint8_t requester_permissions, CayenneLPP& telemetry) override;
  #if ENV_INCLUDE_GPS || defined(ENV_INCLUDE_BME680_BSEC)
  void loop() override;
  #endif
  int getNumSettings() const override;
  const char* getSettingName(int i) const override;
  const char* getSettingValue(int i) const override;
  bool setSettingValue(const char* name, const char* value) override;

  bool hasHardwareInventory() const override { return _i2c_scanned; }
  int  getNumDetectedDevices() const override;
  bool getDetectedDevice(int i, I2CDeviceInfo& out) const override;
  int  getNumActiveSensors() const override { return _active_sensor_count; }
  bool getActiveSensor(int i, I2CDeviceInfo& out) const override;
};
