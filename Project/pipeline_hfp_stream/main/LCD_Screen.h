#ifndef LCD_SCREEN_H
#define LCD_SCREEN_H

#include <Arduino.h>
#include <Arduino_GFX_Library.h>

extern Arduino_GFX *gfx;

void changeUTF(int langCodeInt);
void initLCD();
void setTextLCD(int langCode, String str, int16_t x, int16_t y);
void clearLCD();
void showOpeningScreen();
void showSpeakNowScreen();
void drawMicIcon();
void loopLCD();

#endif // LCD_SCREEN_H
