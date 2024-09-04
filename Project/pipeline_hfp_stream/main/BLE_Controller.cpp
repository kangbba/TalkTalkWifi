#include "BLE_Controller.h"
#include "device_info.h"
#include "Lcd_Screen.h"


bool deviceConnected = false;
bool oldDeviceConnected = false;

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
BLECharacteristic *pRxCharacteristic;
uint8_t txValue = 0;

void MyServerCallbacks::onConnect(BLEServer* pServer) {
    deviceConnected = true;
}

void MyServerCallbacks::onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
}

void MyCallbacks::onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0) {
        Serial.println("********");
        Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++) {
            Serial.print(rxValue[i]);
        }
        Serial.println();

        String fullMsg = rxValue.c_str();

        Serial.print("Received Value to String : ");
        Serial.println();
        Serial.print(fullMsg);

        if (fullMsg.length() > 0) {
          if(fullMsg == "/micOn"){
            setScreen(SCREEN_MIC);
          }
          else if(fullMsg == "/speakerOn"){
            Serial.println("지금 스피커를 켜겠습니다.");
          }
          else if(fullMsg.indexOf(":") != -1 && fullMsg.indexOf(";") != -1)
          {
            int langCode;
            String someMsg;
            parseLangCodeAndMessage(fullMsg, langCode, someMsg);
            if (langCode == 5) {
                someMsg = replaceChinesePunctuations(someMsg);
            } 
            setContentStr(langCode, someMsg);
            setScreen(SCREEN_CONTENT);
          }
        } else {
            Serial.println("Invalid input format. It should be in the format 'langcode:someMsg;'");
        }
    }
}

void sendBLEMessage(const String &data) {
  Serial.println("실제 sendBLEMessage 발동");
  Serial.println(data);
  // 문자열 데이터를 바이트 배열로 변환하여 전송
  uint8_t* byteArray = (uint8_t*)data.c_str();
  size_t byteLength = data.length();
  
  pTxCharacteristic->setValue(byteArray, byteLength);
  pTxCharacteristic->notify();
}

//BLE Functions
void initBLE() {
  Serial.println("Initializing BLE...");
  // Create the BLE Device
  BLEDevice::init(FULL_DEVICE_NAME);
  Serial.println("BLE Device initialized");
  // Create the BLE Server
  pServer = BLEDevice::createServer();
 // pServer->setMtu(256);
  pServer->setCallbacks(new MyServerCallbacks());
  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);
  Serial.println("BLE Server created");

  //TX
  pTxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE
                      );
  pTxCharacteristic->addDescriptor(new BLE2902());

  //RX
  pRxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_RX,
                        BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE
                      );
  pRxCharacteristic->setCallbacks(new MyCallbacks());



  // Start the service
  pService->start();

  Serial.println("TX Characteristic created");
  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}

void loopBLE() {
    if (deviceConnected) {
        pRxCharacteristic->setValue(&txValue, 1);
        pRxCharacteristic->notify();
        txValue++;
        vTaskDelay(10 / portTICK_PERIOD_MS); // FreeRTOS delay
    }
    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        vTaskDelay(500 / portTICK_PERIOD_MS); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
        Serial.print("연결완료");
        clearSerialBuffer();
    }
}
void clearSerialBuffer() {
  while (Serial.available() > 0) {
    Serial.read();
  }
}
void parseLangCodeAndMessage(String input, int &langCode, String &someMsg) {
  int separatorIndex = input.indexOf(":");
  langCode = input.substring(0, separatorIndex).toInt();
  someMsg = input.substring(separatorIndex + 1, input.indexOf(";"));
}

String replaceChinesePunctuations(String str) {
  const char* punctuations[] = {"，", "。", "！", "？", "；", "：", "、", "（", "）"};
  const char* punctuationsForReplace[] = {", ", ".", "!", "?", ";", ":", "、", "(", ")"};
  for (int i = 0; i < sizeof(punctuations) / sizeof(punctuations[0]); i++) {
    str.replace(punctuations[i], punctuationsForReplace[i]);
  }
  return str;
}
