#pragma once
#include <Arduino.h>
#include "control_engine.h"

/**
 * @brief Initializes ESP-NOW receiver on Controller node.
 * Automatically hooks incoming DO telemetry packets directly into the control engine.
 */
bool espNowRxInit(ControlEngine& engine);
