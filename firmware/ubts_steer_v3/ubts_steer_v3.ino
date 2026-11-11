// UBTS front-wheel steering firmware V3 for the NodeMCU ESP8266 (docs/steering/ESP_PROTOCOL_V3_DRAFT_KO.md).
// Drive: 2 drivers (left, right) on D8/D5 and D3/D0, enable D1. Steering: driver on D6/D7, enable D2,
// potentiometer on A0. All parameter values are placeholders until the vehicle specification arrives.
// setup() and loop() stay thin; the logic is in the fw_*.cpp modules of this folder.
#include <Arduino.h>

#include "fw_main.h"

void setup() { ubts_fw::firmware_setup(); }

void loop() { ubts_fw::firmware_loop(); }
