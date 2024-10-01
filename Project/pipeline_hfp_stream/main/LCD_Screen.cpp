#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include "LCD_Screen.h"
#include "Device_Info.h"
#include "background_img.h"
#include "mic_img.h"

#define GFX_BL DF_GFX_BL // default backlight pin, you may replace DF_GFX_BL to actual backlight pin

Arduino_DataBus *bus = create_default_Arduino_DataBus();
Arduino_GFX *gfx = new Arduino_ILI9341(bus, DF_GFX_RST, 3 /* rotation */, false /* IPS */);

#define LCD_TEXT_COLOR MAROON // Purple -> 그린색나옴 ORANGE -> 파란색나옴 CYAN -> 빨강나옴 MAROON -> 하늘색 나옴 DARKGREEN -> 연보라
#define LCD_BACKGROUND_COLOR WHITE

#define LCD_SLEEP_ENABLED false
int curLangCode = 0;
String curContentStr = "";

// 화면 모드를 정의하는 const int 상수
const int SCREEN_MAIN = 0;
const int SCREEN_CONTENT = 1;
const int SCREEN_MIC = 2;
const int SCREEN_CONNECTED = 3;

// "Recording ..." 문자열을 캐싱해두는 전역 변수
const String recordingStr = "Recording ...";
String curAnimatedStr = ""; // 현재 출력 중인 애니메이션 문자열
int recordingIndex = 0; // 현재 출력할 문자 인덱스
const int RECORDING_DELAY_MS = 500; // 각 문자가 출력될 주기 (밀리초)

// 현재 화면 타입 변수
int currentScreen = 0;
int taskIndex = -1;

#define GPIO_BACKLIGHT_PIN 22  // 백라이트 핀을 GPIO 22번으로 설정

TaskHandle_t lcdTaskHandle = NULL; // LCD 백라이트를 제어하는 태스크 핸들

// LCD 상태 설정 함수 (LCD ON/OFF)
void setLcdOn(bool b) {
    if (b) {
        digitalWrite(GPIO_BACKLIGHT_PIN, HIGH);  // LCD 핀을 HIGH로 설정
        Serial.println("LCD ON (HIGH)");
    } else {
        digitalWrite(GPIO_BACKLIGHT_PIN, LOW);   // LCD 핀을 LOW로 설정
        Serial.println("LCD OFF (LOW)");
    }
}

/// FreeRTOS 태스크 함수
void lcdTimerTask(void *param) {
    // 포인터를 통해 전달된 durationSec 값을 가져옴
    if(!LCD_SLEEP_ENABLED){
        return;
    }
    int durationSec = *((int *)param);
    int previousTaskIndex = taskIndex;

    setLcdOn(true);  // LCD ON
    Serial.print("LCD will be on for ");
    Serial.print(durationSec);
    Serial.println(" seconds");

    // 주어진 시간 동안 대기 (vTaskDelay는 1 tick = 1 ms)
    vTaskDelay(durationSec * 1000 / portTICK_PERIOD_MS);

    if (previousTaskIndex == taskIndex) {
        setLcdOn(false);  // LCD OFF
        Serial.println("LCD OFF");
    }

    // 동적 메모리 해제
    free(param);
    vTaskDelete(NULL);  // 태스크 종료
}

// 화면 전환 함수
void setScreen(int screen, int durationSec) {
    clearLCD();
    currentScreen = screen;
    recordingIndex = 0; // 화면이 변경될 때 인덱스를 초기화
    curAnimatedStr = ""; // 애니메이션 문자열 초기화
    switch (screen) {
        case SCREEN_MAIN:
            showOpeningScreen();
            changeUTF(-1);
            gfx->setTextColor(RGB565_BLACK); // Set text color
            gfx->setTextSize(1);
            gfx->setCursor(282, 230);
            gfx->print(FIRMWARE_ID);
            break;
        case SCREEN_MIC:
            drawMicIcon();
            changeUTF(-1);
            gfx->setTextColor(RGB565_BLACK); // Set text color
            gfx->setTextSize(1);
            gfx->setCursor(102, 170);
            gfx->println("SPEAKING NOW");
            break;
        case SCREEN_CONTENT:
            changeUTF(curLangCode);
            gfx->setTextSize(2);
            showCurContent();
            break;
        case SCREEN_CONNECTED:
            showOpeningScreen();
            changeUTF(-1);
            gfx->setTextColor(RGB565_BLACK); // Set text color
            gfx->setTextSize(1);
            gfx->setCursor(282, 230);
            gfx->print(FIRMWARE_ID);
            gfx->setTextColor(RGB565_BLACK); // Set text color
            gfx->setTextSize(1);
            gfx->setCursor(0, 230);
            gfx->print("CONNECTED ✔");
            break;
        default:
            clearLCD(); // 기본 클리어 화면
            break;
    }

    if(LCD_SLEEP_ENABLED){
        // taskIndex 증가
        taskIndex++;
        int *durationSecPtr = (int *)malloc(sizeof(int));
        *durationSecPtr = durationSec;
        // 새로운 태스크 생성 (비동기적으로 실행)
        xTaskCreatePinnedToCore(
            lcdTimerTask,              // 실행할 태스크 함수
            "LCD Task",           // 태스크 이름
            2048,                 // 스택 크기 (바이트 단위)
            durationSecPtr,       // 전달할 인자 (동적 메모리로 할당된 인자)
            1,                    // 우선순위
            NULL,                 // 태스크 핸들 (필요하지 않음)
            1                     // 실행할 코어 (ESP32는 듀얼 코어, 0 또는 1)
        );

    }

}

void setContentStr(int langCode, String contentStr){
    curLangCode = langCode;
    curContentStr = contentStr;
}

// 초기화 함수
void initLCD() {
    // Initialize TFT display
    pinMode(GPIO_BACKLIGHT_PIN, OUTPUT);  // 백라이트 핀을 출력 모드로 설정
    if (!gfx->begin()) {
        Serial.println("gfx->begin() failed!");
    }
    gfx->fillScreen(LCD_BACKGROUND_COLOR);
    gfx->setUTF8Print(true); // enable UTF8 support for the Arduino print() function
#ifdef GFX_BL
    pinMode(GFX_BL, OUTPUT);
    digitalWrite(GFX_BL, HIGH);
#endif
    clearLCD();
    setLcdOn(true);
    setScreen(SCREEN_MAIN, 10); // 시작 시 기본 화면을 main으로 설정
    // LCD를 5초 동안 켰다가 자동으로 끄기
}

// Opening 화면을 보여주는 함수
void showOpeningScreen() {
    gfx->draw16bitRGBBitmap(82, 76, background_img, 150, 80);
}

// Mic 화면을 보여주는 함수
void drawMicIcon() {
    gfx->draw16bitRGBBitmap(82, 44, micicon_img, 150, 100);
}

// Content 화면을 보여주는 함수
void showCurContent() {
    gfx->setTextColor(LCD_TEXT_COLOR); // Set text color
    gfx->setTextSize(2);
    gfx->setCursor(20, 70);
    gfx->println(curContentStr);
    Serial.println(curContentStr);
}

// 화면을 클리어하는 함수
void clearLCD() {
    gfx->fillRect(0, 0, gfx->width(), gfx->height(), LCD_BACKGROUND_COLOR); // Clear the screen
}

// LCD 관련 작업을 loopLCD에서 관리
void loopLCD() {
    // 현재 화면 모드를 관리하는 루프에서 switch 사용
    switch (currentScreen) {
        case SCREEN_MAIN:
            // main 화면의 루프 작업 추가 가능
            break;
         case SCREEN_MIC:
            break;
        case SCREEN_CONTENT:
            // content 화면의 루프 작업 추가 가능
            break;
        default:
            break;
    }
}

// UTF-8 폰트를 설정하는 함수
void changeUTF(int langCodeInt) {
    switch (langCodeInt) {
        case 0: // System
            gfx->setFont(u8g2_font_ncenR12_tr);
            break;
        case 1: // English
         //   gfx->setFont(u8g2_font_ncenR12_tr);
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
        case 10: // Japanese
            gfx->setFont(u8g2_font_b16_t_japanese2);
            break;
        case 12: // Korean
            gfx->setFont(u8g2_korea_kang4); 
            break;
        case 15: // Polish
            gfx->setFont(u8g2_font_unifont_t_polish);
            break;
        default:
            gfx->setFont(u8g2_font_8x13_m_symbols); 
            break;
    }
}
