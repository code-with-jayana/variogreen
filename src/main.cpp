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
String activeFaultReason = "NONE";
bool targetZones[5] = {false, false, false, false, false};
float targetPsi = 0.0f;
unsigned long stateStartTime = 0;
unsigned long lastTelemetryTime = 0;
uint32_t telemetrySeq = 0;
bool wasConnected = false;

void disableAllActuators() {
  digitalWrite(PIN_COMPRESSOR_A0, LOW);
  for (uint8_t i = 0; i < 5; i++) {
    digitalWrite(ZONE_PINS[i], LOW);
  }
  digitalWrite(PIN_EXHAUST_A6, LOW);
}

void triggerFault(const String& reason) {
  disableAllActuators();
  currentState = STATE_FAULT;
  activeFaultReason = reason;
  digitalWrite(PIN_FAULT_LED, HIGH);
  telemetrySeq++;
  bleSendTelemetry("#" + String(telemetrySeq) + "|FAULT:" + reason + "|LOCKOUT:ACTIVE");
}

float readFilteredPsi() {
  static float emaAdc = -1.0f;

  uint32_t sum = 0;
  for (uint8_t i = 0; i < ADC_BURST_SAMPLES; i++) {
    sum += analogRead(PIN_PRESSURE_ADC);
  }
  float rawAvg = static_cast<float>(sum) / ADC_BURST_SAMPLES;

  if (emaAdc < 0.0f) {
    emaAdc = rawAvg;
  } else {
    emaAdc = (ADC_EMA_ALPHA * rawAvg) + ((1.0f - ADC_EMA_ALPHA) * emaAdc);
  }

  if (emaAdc <= ADC_RAW_MIN) return 0.0f;
  if (emaAdc >= ADC_RAW_MAX) return MAX_SIMULATED_PSI;

  float normalized = (emaAdc - ADC_RAW_MIN) / static_cast<float>(ADC_RAW_MAX - ADC_RAW_MIN);
  return normalized * MAX_SIMULATED_PSI;
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

void sendPacket(const String& payload) {
  telemetrySeq++;
  bleSendTelemetry("#" + String(telemetrySeq) + "|" + payload);
}

bool parsePresetCommand(const String& cmd, bool outZones[5], float& outPsi) {
  if (!cmd.startsWith("P:") && !cmd.startsWith("p:")) return false;

  int z[5] = {0};
  float psi = 0.0f;
  char extra = '\0';

  int parsed = sscanf(
    cmd.c_str() + 2,
    "%d,%d,%d,%d,%d,%f%c",
    &z[0], &z[1], &z[2], &z[3], &z[4], &psi, &extra
  );
  if (parsed != 6) return false;

  bool anyZoneActive = false;
  for (int i = 0; i < 5; i++) {
    if (z[i] != 0 && z[i] != 1) return false;
    outZones[i] = (z[i] == 1);
    if (outZones[i]) anyZoneActive = true;
  }

  if (!anyZoneActive) return false;
  if (isnan(psi) || isinf(psi) || psi <= EXHAUST_DONE_PSI || psi > MAX_SIMULATED_PSI) {
    return false;
  }

  outPsi = psi;
  return true;
}

 void handleCommand(const String& cmd) {
  // 1. STOP
  if (cmd.equalsIgnoreCase("STOP")) {
    triggerFault("EMERGENCY_STOP");
    return;
  }

  // 2. Clear fault
  if (cmd.equalsIgnoreCase("CLEAR")) {
    if (currentState != STATE_FAULT) {
      sendPacket("ACK:CLEAR|NO_ACTIVE_FAULT");
      return;
    }

    disableAllActuators();
    digitalWrite(PIN_FAULT_LED, LOW);
    activeFaultReason = "NONE";
    currentState = STATE_IDLE;
    sendPacket("ACK:CLEAR|LOCKOUT:RELEASED|STATE:IDLE");
    return;
  }

  // 3. Status query
  if (cmd.equalsIgnoreCase("STAT")) {
    float psi = readFilteredPsi();

    sendPacket("ACK:STAT|STATE:" +
               String(stateToString(currentState)) +
               "|PSI:" + String(psi, 1) +
               "|FAULT:" + activeFaultReason);
    return;
  }

  // 4. Reject actuator commands during a fault
  if (currentState == STATE_FAULT) {
    sendPacket("NACK:LOCKED_OUT|REASON:" +
               activeFaultReason + "|SEND_CLEAR");
    return;
  }

  // 5. Reset / deflate
  if (cmd.equalsIgnoreCase("RST")) {
    // Repeated RST must not restart the timer
    if (currentState == STATE_DEFLATING) {
      sendPacket("ACK:RST|ALREADY_DEFLATING");
      return;
    }

    disableAllActuators();
    digitalWrite(PIN_EXHAUST_A6, HIGH);
    currentState = STATE_DEFLATING;
    stateStartTime = millis();

    sendPacket("ACK:RST|STATE:DEFLATING");
    return;
  }

  // 6. Preset inflation
  if (cmd.startsWith("P:") || cmd.startsWith("p:")) {
    // New presets must not restart an active timer
    if (currentState == STATE_INFLATING ||
        currentState == STATE_DEFLATING) {
      sendPacket("NACK:BUSY");
      return;
    }

    bool parsedZones[5];
    float parsedPsi = 0.0f;

    if (!parsePresetCommand(cmd, parsedZones, parsedPsi)) {
      sendPacket("NACK:INVALID_PRESET_SYNTAX");
      return;
    }

    for (uint8_t i = 0; i < 5; i++) {
      targetZones[i] = parsedZones[i];
    }
    targetPsi = parsedPsi;

    disableAllActuators();

    for (uint8_t i = 0; i < 5; i++) {
      digitalWrite(ZONE_PINS[i], targetZones[i] ? HIGH : LOW);
    }

    stateStartTime = millis();
    digitalWrite(PIN_COMPRESSOR_A0, HIGH);
    currentState = STATE_INFLATING;

    sendPacket("ACK:PRESET|STATE:INFLATING|TARGET:" +
               String(targetPsi, 1));
    return;
  }

  sendPacket("NACK:UNKNOWN_CMD");
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_COMPRESSOR_A0, OUTPUT);
  for (uint8_t i = 0; i < 5; i++) {
    pinMode(ZONE_PINS[i], OUTPUT);
  }
  pinMode(PIN_EXHAUST_A6, OUTPUT);
  pinMode(PIN_FAULT_LED, OUTPUT);

  disableAllActuators();
  digitalWrite(PIN_FAULT_LED, LOW);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_PRESSURE_ADC, ADC_11db);
  pinMode(PIN_PRESSURE_ADC, INPUT);

  bleInit("PuttingGreen_Ctrl");
}

void loop() {
  bool connected = bleIsConnected();

  if (wasConnected && !connected && currentState == STATE_INFLATING) {
    triggerFault("BLE_LINK_LOST");
  }
  wasConnected = connected;

// Check timeout before processing an ordinary command
unsigned long now = millis();

bool timedState =
  currentState == STATE_INFLATING ||
  currentState == STATE_DEFLATING;

if (timedState && now - stateStartTime >= SAFETY_TIMEOUT_MS) {
  triggerFault(
    currentState == STATE_INFLATING
      ? "TIMEOUT_TBR01"
      : "DEFLATE_TIMEOUT"
  );
}

// Process at most one ordinary command per loop
String cmd;
if (bleReceiveCommand(cmd)) {
  Serial.println("[RX] " + cmd);
  handleCommand(cmd);
}

float currentPsi = readFilteredPsi();
now = millis();

  switch (currentState) {
    case STATE_INFLATING:
      if (now - stateStartTime >= SAFETY_TIMEOUT_MS) {
        triggerFault("TIMEOUT_TBR01");
      } else if (currentPsi >= targetPsi) {
        disableAllActuators();
        currentState = STATE_LOCKED;
        sendPacket("EVENT:preset-complete|PSI:" + String(currentPsi, 1));
      }
      break;

    case STATE_DEFLATING:
      if (currentPsi <= EXHAUST_DONE_PSI) {
        disableAllActuators();
        currentState = STATE_IDLE;
        sendPacket("EVENT:reset-complete|PSI:" + String(currentPsi, 1));
      } else if (now - stateStartTime >= SAFETY_TIMEOUT_MS) {
        triggerFault("DEFLATE_TIMEOUT");
      }
      break;

    case STATE_IDLE:
    case STATE_LOCKED:
    case STATE_FAULT:
      break;
  }

  if (connected && (now - lastTelemetryTime >= 1000)) {
    lastTelemetryTime = now;
    sendPacket("HB|STATE:" + String(stateToString(currentState)) + "|PSI:" + String(currentPsi, 1));
  }

  delay(10);
}