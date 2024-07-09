//[LCD]
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <BluetoothSerial.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Arduino_GFX_Library.h>
#include "u8g2_korea_kang4.h"


//[ADF HFP]
#include "esp_bt.h"
#include "esp_bt_device.h"
#include <string.h>
#include <inttypes.h>
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_peripherals.h"
#include "periph_touch.h"
#include "periph_adc_button.h"
#include "periph_button.h"
#include "esp_bt_defs.h"
#include "esp_gap_bt_api.h"
#include "esp_hf_client_api.h"
#include "audio_element.h"
#include "audio_pipeline.h"
#include "audio_event_iface.h"
#include "audio_mem.h"
#include "i2s_stream.h"
#include "board.h"
#include "bluetooth_service.h"
#include "filter_resample.h"
#include "raw_stream.h"
#if (CONFIG_ESP_LYRATD_MSC_V2_1_BOARD || CONFIG_ESP_LYRATD_MSC_V2_2_BOARD)
#include "filter_resample.h"
#endif
#include "bt_keycontrol.h"
#include "audio_idf_version.h"
#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0))
#define HFP_RESAMPLE_RATE 16000
#else
#define HFP_RESAMPLE_RATE 8000
#endif

#define DEVICE_NAME "TamiOn_3016"

// Function declarations
void initBLEDevice();
void clearSerialBuffer();
void openingMent();
void connectedMent();
void scrollWithInterval(long interval);
void parseLangCodeAndMessage(String input, int &langCode, String &someMsg);
String replaceChinesePunctuations(String str);
void Message(int langCode, String str);
void ChangeUTF(int langCodeInt);


// Bluetooth
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define GFX_BL DF_GFX_BL // default backlight pin, you may replace DF_GFX_BL to actual backlight pin

Arduino_DataBus *bus = create_default_Arduino_DataBus();
Arduino_GFX *gfx = new Arduino_ILI9341(bus, DF_GFX_RST, 1 /* rotation */, false /* IPS */);

#define SERVICE_UUID           "4fafc201-1fb5-459e-8fcc-c5c9c331914b" // UART service UUID
#define CHARACTERISTIC_UUID_RX "beb5483e-36e1-4688-b7f5-ea07361b26a8" // 
#define CHARACTERISTIC_UUID_TX "beb5483e-36e1-4688-b7f5-ea07361b26a9" // 

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;
String recentMessage = "";

int nowCallback = 0;
int previousCallback = 0;

int deviceWidth = 128;

int maxCursorY = 0;
int currentCursorY = 0;

int gapWithTextLines = 24;

// 긴 텍스트를 위한 스크롤 기능
unsigned long scrollStartTime = 0;
unsigned long accumTimeForScroll = 0;
long previousMillis = 0; 

// 아래의 두개 수정가능
int scrollStartDelayTime = 3000; // (3000이면 3초 있다가 스크롤 시작)
int scrollDelay = 200; // (이 값이 클수록 스크롤 속도가 느려짐)

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();
      if (rxValue.length() > 0) {
        Serial.println("********");
        Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i]);

        Serial.println();

        recentMessage = rxValue.c_str(); 

        Serial.print("Received Value to String : ");
        Serial.print(recentMessage);
        Serial.println();

        nowCallback++;

        
        if (recentMessage.indexOf(":") != -1 && recentMessage.indexOf(";") != -1) {
          int langCode;
          String someMsg;
          parseLangCodeAndMessage(recentMessage, langCode, someMsg);

          if (langCode == 5) {
            someMsg = replaceChinesePunctuations(someMsg);
          }
          Message(langCode, someMsg);
        } else {
          Serial.println("Invalid input format. It should be in the format 'langcode:someMsg;'");
        }
      }
    }

};

void setup() {
  Serial.begin(115200);
  Serial.println("Arduino_GFX U8g2 Font UTF8 Full Unifont example");

  
  // Create the BLE Device
  initBLEDevice();
  // Initialize TFT display
  if (!gfx->begin()) {
    Serial.println("gfx->begin() failed!");
  }
  gfx->fillScreen(YELLOW);
  gfx->setUTF8Print(true); // enable UTF8 support for the Arduino print() function

#ifdef GFX_BL
  pinMode(GFX_BL, OUTPUT);
  digitalWrite(GFX_BL, HIGH);
#endif

  gfx->setTextColor(BLACK); //TEXT COLOR -> BLACK
  gfx->setTextSize(2);

  clearSerialBuffer();

  openingMent();
}

void initBLEDevice() {
  // Create the BLE Device
  BLEDevice::init("TamiOn");
  // Create the BLE Server
  pServer = BLEDevice::createServer();
 // pServer->setMtu(256);
  pServer->setCallbacks(new MyServerCallbacks());
  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);
  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
                    CHARACTERISTIC_UUID_TX,
                    BLECharacteristic::PROPERTY_NOTIFY
                  ); 
  pTxCharacteristic->addDescriptor(new BLE2902());
  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
                       CHARACTERISTIC_UUID_RX,
                      BLECharacteristic::PROPERTY_WRITE
                    );
  pRxCharacteristic->setCallbacks(new MyCallbacks());
  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}

void openingMent() {
  String str =  "Talk Talk WIFI";
  gfx->fillScreen(YELLOW);
  gfx->setCursor(20, 120);
  gfx->setFont(u8g2_font_ncenR12_tr); // 한국어 (KANG)
  gfx->println(str);
}

void connectedMent() {
  String str2 =  "Device Connected";
  gfx->fillScreen(YELLOW);
  gfx->setCursor(80, 120);
  gfx->setFont(u8g2_korea_kang4); // 한국어 (KANG)
  gfx->println(str2);
}

void loop() {
  unsigned long currentMillis = millis();
  unsigned long deltaTime = currentMillis - previousMillis;
  
  if (deviceConnected) {
      pTxCharacteristic->setValue(&txValue, 1);
      pTxCharacteristic->notify();
      txValue++;
      delay(10); // bluetooth stack will go into congestion, if too many packets are sent
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
      delay(500); // give the bluetooth stack the chance to get things ready
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

  bool recentMessageExist = nowCallback != previousCallback;
  if (recentMessageExist) {
    currentCursorY = 0;
    maxCursorY = 0;
    previousCallback = nowCallback;
    accumTimeForScroll = 0;
  }


  if (accumTimeForScroll >= scrollStartDelayTime)
    scrollWithInterval(scrollDelay);
  else {
    accumTimeForScroll += deltaTime;
  }
  previousMillis = currentMillis;
}

void clearSerialBuffer() {
  while (Serial.available() > 0) {
    Serial.read();
  }
}

void scrollWithInterval(long interval) {
  unsigned long currentMillis = millis();

  // check if it's time to scroll
  if (currentMillis - scrollStartTime >= interval) {
    // do scrolling here
    int maxLineCount = 4; // 수정가능
    int onePageHeight = gapWithTextLines * maxLineCount;
    if (maxCursorY >= onePageHeight && currentCursorY < (maxCursorY - onePageHeight)) {
      currentCursorY++;
    } else {
      // reset scrollStartTime to currentMillis so the next scroll interval starts from now
      scrollStartTime = currentMillis;
    }
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

void Message(int langCode, String str) {
  gfx->fillRect(0, 0, gfx->width(), gfx->height(), YELLOW); // 필요할 때만 전체 화면을 지우기
  ChangeUTF(langCode);
  gfx->setCursor(10, 50);
  gfx->println(str);
  Serial.println(str);
}

void ChangeUTF(int langCodeInt) {
  switch (langCodeInt) {
    case 1: // English
        gfx->setFont(u8g2_font_ncenR12_tr);
        break;
    case 2: // Spanish
    case 3: // French
    case 4: // German
        gfx->setFont(u8g2_font_7x14_tf);
        break;
    case 5: // Chinese
         gfx->setFont(u8g2_font_wqy14_t_gb2312a);
        break;
    case 6: // Arabic
        gfx->setFont(u8g2_font_cu12_t_arabic); 
        break;
    case 7: // Russian
        gfx->setFont(u8g2_font_cu12_t_cyrillic);
        break;
    case 8: // Portuguese
    case 11: // Dutch
    case 22: // Hungarian
    case 24: // Romanian
    case 41: // Indonesian
        gfx->setFont(u8g2_font_7x14_tf);
        break;
    case 9: // Italian
    case 10: // Japanese
        gfx->setFont(u8g2_font_b16_t_japanese2);
        break;
    case 12: // Korean
        gfx->setFont(u8g2_korea_kang4);
        break;
    case 13: // Swedish
    case 14: // Turkish
    case 15: // Polish
    case 16: // Danish
    case 17: // Norwegian
    case 18: // Finnish
    case 19: // Czech
    case 23: // Hebrew
    case 25: // Ukrainian
    case 27: // Icelandic
    case 28: // Bulgarian
    case 29: // Lithuanian
    case 30: // Latvian
    case 31: // Slovenian
    case 32: // Croatian
    case 33: // Estonian
        gfx->setFont(u8g2_font_7x14_tf);
        break;
    case 20: // Thai
        gfx->setFont(u8g2_font_etl24thai_t);
        break;
    case 21: // Greek
        gfx->setFont(u8g2_font_unifont_t_greek);
        break;
    case 26: // Vietnamese
        gfx->setFont(u8g2_font_unifont_t_vietnamese2);
        break;
    default:
        gfx->setFont(u8g2_font_unifont_t_symbols);
        break;
  }
}

extern "C" void app_main() {

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize the Arduino framework
    initArduino();

    // Call the setup function
    setup();

    // Enter the loop
    while (true) {
        loop();
        delay(10); // Add a small delay to prevent the loop from running too fast
    }
}
