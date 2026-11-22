#pragma once
#include <Arduino.h>

// Actuator Output Channels (Safe non-strapping GPIOs)
constexpr uint8_t PIN_COMPRESSOR_A0 = 25; // Yellow LED: Compressor Relay
constexpr uint8_t PIN_ZONE_A1       = 26; // Green LED 1: Wedge Zone 1
constexpr uint8_t PIN_ZONE_A2       = 27; // Green LED 2: Wedge Zone 2
constexpr uint8_t PIN_ZONE_A3       = 14; // Green LED 3: Wedge Zone 3
constexpr uint8_t PIN_ZONE_A4       = 13; // Green LED 4: Wedge Zone 4
constexpr uint8_t PIN_ZONE_A5       = 33; // Green LED 5: Wedge Zone 5
constexpr uint8_t PIN_EXHAUST_A6    = 32; // Red LED: Common Exhaust Vent
constexpr uint8_t PIN_FAULT_LED     = 23; // Fault Indicator LED

const uint8_t ZONE_PINS[5] = {
  PIN_ZONE_A1, PIN_ZONE_A2, PIN_ZONE_A3, PIN_ZONE_A4, PIN_ZONE_A5
};

// Analog Pressure Sensor Input (ADC1 - safe to use alongside BLE)
constexpr uint8_t PIN_PRESSURE_ADC  = 34;

// System & Safety Constants (TBR01)
constexpr unsigned long SAFETY_TIMEOUT_MS = 90000; // 90-second max run-time
constexpr float MAX_SIMULATED_PSI         = 30.0f; // Full-scale simulated PSI
constexpr float EXHAUST_DONE_PSI          = 0.5f;  // Baseline threshold for reset complete

// ADC Calibration & Filtering Constants
constexpr int ADC_RAW_MIN                 = 60;    // Deadband floor (clamps pot noise to 0.0 PSI)
constexpr int ADC_RAW_MAX                 = 4035;  // Deadband ceiling (clamps near 3.3V to max PSI)
constexpr uint8_t ADC_BURST_SAMPLES       = 16;    // Oversampling count per loop tick
constexpr float ADC_EMA_ALPHA             = 0.25f; // Exponential Moving Average smoothing factor