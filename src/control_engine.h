#pragma once
#include <Arduino.h>

enum ControlMode {
  MODE_PID = 0,
  MODE_FUZZY = 1,
  MODE_MANUAL = 2
};

struct ControlConfig {
  ControlMode mode = MODE_PID;
  float target_do_mg_l = 7.5f;
  float kp = 100.0f;
  float ki = 1.5f;
  float kd = 10.0f;
  float min_rpm = 0.0f;
  float max_rpm = 600.0f;
  float failsafe_rpm = 0.0f;
  float manual_throttle_pct = 0.0f;
  uint32_t config_version = 1;
};

class ControlEngine {
public:
  ControlEngine();

  // Thread-safe configuration read/write across cores
  void configure(const ControlConfig& config);
  ControlConfig getConfig();

  // Thread-safe command helpers called from Core 0
  void setManualThrottle(float throttlePct);
  void triggerEmergencyStop();
  bool isEmergencyStopActive();
  void clearEmergencyStop();

  // Called from ESP-NOW receiver
  void updateProcessVariable(float measuredDoMgL, float waterTempC);

  // Called periodically on Core 1 (20 Hz)
  float computeOutputRpm(unsigned long nowMs);

  bool isFailsafeActive();
  float getLastMeasuredDo();
  float getLastWaterTemp();
  unsigned long getLastPacketAgeMs(unsigned long nowMs);

  // V4: la grabación SD y la ventana de alta frecuencia solo se habilitan durante un experimento.
  void setExperimentLogging(bool active);
  bool isExperimentLogging();

private:
  portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;

  ControlConfig _config;
  
  float _lastMeasuredDo = 0.0f;
  float _lastWaterTemp = 25.0f;
  unsigned long _lastPacketTimeMs = 0;
  bool _hasFirstSample = false;
  bool _failsafeActive = false;
  bool _emergencyStopActive = false;
  bool _experimentLogging = false;

  // PID State (accessed only from Core 1)
  float _integral = 0.0f;
  float _prevError = 0.0f;
  unsigned long _lastComputeMs = 0;

  float runPid(float error, float dt, const ControlConfig& cfg);
  float runFuzzy(float error, float dError, const ControlConfig& cfg);
};
