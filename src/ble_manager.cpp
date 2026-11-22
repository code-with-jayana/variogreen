#include "ble_manager.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <atomic>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

constexpr size_t MAX_CMD_LEN   = 48;
constexpr size_t CMD_QUEUE_LEN = 8;

static BLECharacteristic* pTxCharacteristic = nullptr;
static std::atomic<bool> deviceConnected{false};
static QueueHandle_t rxCommandQueue = nullptr;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    deviceConnected.store(true);
    Serial.println("[BLE] Client connected");
  }

  void onDisconnect(BLEServer* pServer) override {
    deviceConnected.store(false);
    if (rxCommandQueue != nullptr) {
      xQueueReset(rxCommandQueue);
    }
    Serial.println("[BLE] Client disconnected - advertising restarted");
    pServer->getAdvertising()->start();
  }
};

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    String rxValue = pCharacteristic->getValue();
    rxValue.trim();

    if (rxValue.length() == 0 || rxValue.length() >= MAX_CMD_LEN) {
      Serial.println("[BLE] Dropped empty or oversized packet");
      return;
    }

    char cmdBuffer[MAX_CMD_LEN] = {0};
    rxValue.toCharArray(cmdBuffer, MAX_CMD_LEN);

    if (rxCommandQueue != nullptr) {
      if (xQueueSend(rxCommandQueue, cmdBuffer, 0) != pdTRUE) {
        Serial.println("[BLE] RX queue full - command dropped");
      }
    }
  }
};

void bleInit(const char* deviceName) {
  rxCommandQueue = xQueueCreate(CMD_QUEUE_LEN, MAX_CMD_LEN);

  BLEDevice::init(deviceName);
  BLEServer* pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  pTxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic* pRxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  pRxCharacteristic->setCallbacks(new RxCallbacks());

  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("[BLE] Thread-safe GATT server advertising");
}

bool bleIsConnected() {
  return deviceConnected.load();
}

bool bleReceiveCommand(String& outCommand) {
  if (rxCommandQueue == nullptr) return false;

  char cmdBuffer[MAX_CMD_LEN] = {0};
  if (xQueueReceive(rxCommandQueue, cmdBuffer, 0) == pdTRUE) {
    outCommand = String(cmdBuffer);
    return true;
  }
  return false;
}

bool bleSendTelemetry(const String& message) {
  Serial.println("[TX] " + message);
  if (!deviceConnected.load() || pTxCharacteristic == nullptr) {
    return false;
  }

  pTxCharacteristic->setValue(message.c_str());
  pTxCharacteristic->notify();
  return true;
}