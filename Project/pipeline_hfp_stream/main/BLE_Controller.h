#ifndef BLE_CONTROLLER_H
#define BLE_CONTROLLER_H

#include "Arduino.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <string.h>

#define SERVICE_UUID           "4fafc201-1fb5-459e-8fcc-c5c9c331914b" // UART service UUID
#define CHARACTERISTIC_UUID_RX "beb5483e-36e1-4688-b7f5-ea07361b26a8" // 
#define CHARACTERISTIC_UUID_TX "beb5483e-36e1-4688-b7f5-ea07361b26a9" // 

extern bool deviceConnected;
extern bool oldDeviceConnected;
extern BLEServer *pServer;
extern BLECharacteristic *pTxCharacteristic;
extern uint8_t txValue;

void initBLE();
void loopBLE();
void clearSerialBuffer();
void parseLangCodeAndMessage(String input, int &langCode, String &someMsg);
String replaceChinesePunctuations(String str);

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer);
    void onDisconnect(BLEServer* pServer);
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic);
};

#endif // BLE_CONTROLLER_H
