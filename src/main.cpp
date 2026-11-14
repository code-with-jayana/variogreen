#include <Arduino.h>

const int LED_PIN = 2;   // Standard onboard LED on most ESP32 DevKits
const int ADC_PIN = 34;  // ADC1_CH6 (input-only, safe with Wi-Fi/BLE)

void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial monitor time to connect

  pinMode(LED_PIN, OUTPUT);
  analogReadResolution(12); // 0 - 4095

  Serial.printf("Chip Model: %s (Rev %d)\n", ESP.getChipModel(), ESP.getChipRevision());
  Serial.printf("Cores: %d | CPU Freq: %d MHz\n", ESP.getChipCores(), ESP.getCpuFreqMHz());
  Serial.printf("Flash Size: %d MB\n", ESP.getFlashChipSize() / (1024 * 1024));

}

void loop() {
  static bool ledState = false;
  ledState = !ledState;
  digitalWrite(LED_PIN, ledState);

  int rawAdc = analogRead(ADC_PIN);
  float voltage = (rawAdc / 4095.0f) * 3.3f;

  Serial.printf("LED: %s | GPIO 34 Raw: %4d | Est. Voltage: %.2f V\n",
                ledState ? "ON " : "OFF", rawAdc, voltage);

  delay(500);
}