#pragma once
#include <Arduino.h>

void bleInit(const char* deviceName);
bool bleIsConnected();
bool bleHasCommand();
String bleReadCommand();
void bleSendTelemetry(const String& message);