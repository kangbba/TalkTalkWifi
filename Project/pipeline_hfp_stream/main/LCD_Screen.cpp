#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include "LCD_Screen.h"
#include "Device_Info.h"
#include "background_img.h"
#include "mic_img.h"

#define GFX_BL DF_GFX_BL // default backlight pin, you may replace DF_GFX_BL to actual backlight pin

Arduino_DataBus *bus = create_default_Arduino_DataBus();
Arduino_GFX *gfx = new Arduino_ILI9341(bus, DF_GFX_RST, 3 /* rotation */, false /* IPS */);
#define LCD_TEXT_COLOR MAROON //Purple -> 그린색나옴 ORANGE -> 파란색나옴 CYAN -> 빨강나옴 MAROON -> 하늘색 나옴 DARKGREEN -> 연보라
#define LCD_BACKGROUND_COLOR WHITE

/*
#define BLACK RGB565_BLACK
#define NAVY RGB565_NAVY
#define DARKGREEN RGB565_DARKGREEN
#define DARKCYAN RGB565_DARKCYAN
#define MAROON RGB565_MAROON
#define PURPLE RGB565_PURPLE
#define OLIVE RGB565_OLIVE
#define LIGHTGREY RGB565_LIGHTGREY
#define DARKGREY RGB565_DARKGREY
#define BLUE RGB565_BLUE
#define GREEN RGB565_GREEN
#define CYAN RGB565_CYAN
#define RED RGB565_RED
#define MAGENTA RGB565_MAGENTA
#define YELLOW RGB565_YELLOW
#define WHITE RGB565_WHITE
#define ORANGE RGB565_ORANGE
#define GREENYELLOW RGB565_GREENYELLOW
#define PALERED RGB565_PALERED*/

void showSpeakNowScreen(){

}
void changeUTF(int langCodeInt) {
  switch (langCodeInt) {
    case 0: // System
        gfx->setFont(u8g2_font_ncenR12_tr);
        break;
    case 1: // English
        gfx->setFont(u8g2_font_ncenR12_tr);
        break;
    case 2: // Spanish
    case 3: // French
    case 4: // German
         gfx->setFont(u8g2_font_7x14_tf);
        break;
    case 5: // Chinese
        //gfx->setFont(u8g2_font_unifont_t_chinese4);
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
      //  gfx->setFont(u8g2_font_unifont_t_japanese1);
        gfx->setFont(u8g2_font_b16_t_japanese2);
        break;
    case 12: // Korean
        gfx->setFont(u8g2_korea_kang4); 
        //gfx->setFont(u8g2_font_unifont_t_korean1);
      //  gfx->setFont(u8g2_font_ncenR12_tr);
        break;
    case 15: // Polish
        gfx->setFont(u8g2_font_unifont_t_polish);
        break;
    case 9: // Italian
    case 13: // Swedish
    case 14: // Turkish
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
void initLCD() {
    // Initialize TFT display
    if (!gfx->begin()) {
        Serial.println("gfx->begin() failed!");
    }
   // gfx->draw16bitRGBBitmap(0, 0, background_img, 320, 280);
    gfx->fillScreen(LCD_BACKGROUND_COLOR);
    gfx->setUTF8Print(true); // enable UTF8 support for the Arduino print() function
#ifdef GFX_BL
    pinMode(GFX_BL, OUTPUT);
    digitalWrite(GFX_BL, HIGH);
#endif
    clearLCD();
    showOpeningScreen();
}
void showOpeningScreen(){
    gfx->draw16bitRGBBitmap(60, 70, background_img, 200,  87);
   // setTextLCD(0, DEVICE_NAME, 40, 120);
}
void showBackground(){
}
void clearLCD() {
    gfx->fillRect(0, 0, gfx->width(), gfx->height(), LCD_BACKGROUND_COLOR); // Clear the screen
    //gfx->draw16bitRGBBitmap(0, 0, background_img, 320, 280);
}

void setTextLCD(int langCode, String str, int16_t x, int16_t y) {
    gfx->setTextColor(LCD_TEXT_COLOR); // Set text color
    gfx->setTextSize(2);
    changeUTF(langCode);
    gfx->setCursor(x, y);
    gfx->println(str);
    Serial.println(str);
}

void drawMicIcon() {
    gfx->draw16bitRGBBitmap(60, 70, mic_img, 200,  87);
}





