#include "ble_manager.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

static BLECharacteristic* pTxCharacteristic = nullptr;
static bool deviceConnected = false;
static volatile bool newCommandReady = false;
static String incomingCommand = "";

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    deviceConnected = true;
    Serial.println("[BLE] Client connected");
  }
  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
    Serial.println("[BLE] Client disconnected - advertising restarted");
    pServer->getAdvertising()->start();
  }
};

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    String rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0) {
      incomingCommand = rxValue;
      incomingCommand.trim();
      newCommandReady = true;
    }
  }
};

void bleInit(const char* deviceName) {
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
    BLECharacteristic::PROPERTY_WRITE
  );
  pRxCharacteristic->setCallbacks(new RxCallbacks());

  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("[BLE] Initialized and advertising");
}

bool bleIsConnected() {
  return deviceConnected;
}

bool bleHasCommand() {
  return newCommandReady;
}

String bleReadCommand() {
  newCommandReady = false;
  return incomingCommand;
}

void bleSendTelemetry(const String& message) {
  Serial.println("[TX] " + message);
  if (deviceConnected && pTxCharacteristic != nullptr) {
    pTxCharacteristic->setValue(message.c_str());
    pTxCharacteristic->notify();
  }
}