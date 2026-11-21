#include <Arduino.h>
#include "config.h"
#include "ble_manager.h"

enum SystemState {
  STATE_IDLE,
  STATE_INFLATING,
  STATE_LOCKED,
  STATE_DEFLATING,
  STATE_FAULT
};

SystemState currentState = STATE_IDLE;
bool targetZones[5] = {false, false, false, false, false};
float targetPsi = 0.0f;
unsigned long stateStartTime = 0;
unsigned long lastTelemetryTime = 0;

// Helper: Force every actuator LOW (fail-safe)
void disableAllActuators() {
  digitalWrite(PIN_COMPRESSOR_A0, LOW);
  for (uint8_t i = 0; i < 5; i++) {
    digitalWrite(ZONE_PINS[i], LOW);
  }
  digitalWrite(PIN_EXHAUST_A6, LOW);
}

// Helper: Read potentiometer and map 0-4095 -> 0.0-MAX_SIMULATED_PSI
float readSimulatedPsi() {
  int rawAdc = analogRead(PIN_PRESSURE_ADC);
  return (rawAdc / 4095.0f) * MAX_SIMULATED_PSI;
}

const char* stateToString(SystemState s) {
  switch (s) {
    case STATE_IDLE:      return "IDLE";
    case STATE_INFLATING: return "INFLATING";
    case STATE_LOCKED:    return "LOCKED";
    case STATE_DEFLATING: return "DEFLATING";
    case STATE_FAULT:     return "FAULT";
    default:              return "UNKNOWN";
  }
}

// Parse commands:
//   "P:1,0,1,0,0,15.0" -> Activate zones 1 & 3, target 15.0 PSI
//   "RST"              -> Open exhaust A6 until pressure <= EXHAUST_DONE_PSI
//   "STOP"             -> Emergency stop (kills all outputs, enters FAULT)
void handleCommand(const String& cmd) {
  if (cmd.equalsIgnoreCase("STOP")) {
    disableAllActuators();
    currentState = STATE_FAULT;
    bleSendTelemetry("FAULT:EMERGENCY_STOP");
    return;
  }

  if (cmd.equalsIgnoreCase("RST")) {
    disableAllActuators();
    digitalWrite(PIN_EXHAUST_A6, HIGH);
    currentState = STATE_DEFLATING;
    stateStartTime = millis();
    bleSendTelemetry("STATE:DEFLATING");
    return;
  }

  if (cmd.startsWith("P:") || cmd.startsWith("p:")) {
    // Expected format: P:z1,z2,z3,z4,z5,psi (minimum 13 chars)
    if (cmd.length() < 13) {
      bleSendTelemetry("ERR:BAD_FORMAT");
      return;
    }

    for (uint8_t i = 0; i < 5; i++) {
      char val = cmd.charAt(2 + (i * 2));
      targetZones[i] = (val == '1');
    }
    targetPsi = cmd.substring(12).toFloat();

    if (targetPsi <= 0.0f || targetPsi > MAX_SIMULATED_PSI) {
      bleSendTelemetry("ERR:INVALID_PSI");
      return;
    }

    // Engage compressor A0 and selected zone valves A1-A5; ensure exhaust A6 is closed
    disableAllActuators();
    digitalWrite(PIN_COMPRESSOR_A0, HIGH);
    for (uint8_t i = 0; i < 5; i++) {
      digitalWrite(ZONE_PINS[i], targetZones[i] ? HIGH : LOW);
    }

    currentState = STATE_INFLATING;
    stateStartTime = millis();
    bleSendTelemetry("STATE:INFLATING|TARGET:" + String(targetPsi, 1));
    return;
  }

  bleSendTelemetry("ERR:UNKNOWN_CMD");
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_COMPRESSOR_A0, OUTPUT);
  for (uint8_t i = 0; i < 5; i++) {
    pinMode(ZONE_PINS[i], OUTPUT);
  }
  pinMode(PIN_EXHAUST_A6, OUTPUT);
  disableAllActuators();

  pinMode(PIN_PRESSURE_ADC, INPUT);

  bleInit("PuttingGreen_Ctrl");
}

void loop() {
  // 1. Check for incoming BLE commands
  if (bleHasCommand()) {
    String cmd = bleReadCommand();
    Serial.println("[RX] " + cmd);
    handleCommand(cmd);
  }

  float currentPsi = readSimulatedPsi();
  unsigned long now = millis();

  // 2. Run state machine transitions
  switch (currentState) {
    case STATE_INFLATING:
      // Check TBR01 90-second safety timeout first
      if (now - stateStartTime >= SAFETY_TIMEOUT_MS) {
        disableAllActuators();
        currentState = STATE_FAULT;
        bleSendTelemetry("FAULT:TIMEOUT_TBR01");
      }
      // Check closed-loop pressure target reached
      else if (currentPsi >= targetPsi) {
        disableAllActuators();
        currentState = STATE_LOCKED;
        bleSendTelemetry("preset-complete|PSI:" + String(currentPsi, 1));
      }
      break;

    case STATE_DEFLATING:
      // Vent until line pressure returns to baseline (or timeout failsafe)
      if (currentPsi <= EXHAUST_DONE_PSI) {
        disableAllActuators();
        currentState = STATE_IDLE;
        bleSendTelemetry("reset-complete|PSI:" + String(currentPsi, 1));
      } else if (now - stateStartTime >= SAFETY_TIMEOUT_MS) {
        disableAllActuators();
        currentState = STATE_FAULT;
        bleSendTelemetry("FAULT:DEFLATE_TIMEOUT");
      }
      break;

    case STATE_IDLE:
    case STATE_LOCKED:
    case STATE_FAULT:
      // Outputs remain held/off until next command
      break;
  }

  // 3. Stream periodic telemetry every 1 second while connected
  if (bleIsConnected() && (now - lastTelemetryTime >= 1000)) {
    lastTelemetryTime = now;
    String status = String(stateToString(currentState)) + "|PSI:" + String(currentPsi, 1);
    bleSendTelemetry(status);
  }

  delay(20);
}