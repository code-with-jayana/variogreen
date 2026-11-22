#pragma once
#include <Arduino.h>

void bleInit(const char* deviceName);
bool bleIsConnected();
bool bleReceiveCommand(String& outCommand);
bool bleSendTelemetry(const String& message);