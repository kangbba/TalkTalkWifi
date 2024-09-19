#ifndef LCD_SCREEN_H
#define LCD_SCREEN_H

#include <Arduino.h>
#include <Arduino_GFX_Library.h>

// 외부에서 사용할 수 있도록 gfx 객체 선언
extern Arduino_GFX *gfx;

// 화면 모드를 정의하는 const int 상수
extern const int SCREEN_MAIN;
extern const int SCREEN_CONTENT;
extern const int SCREEN_MIC;
extern const int SCREEN_CONNECTED;

// 현재 화면 타입 변수 선언
extern int currentScreen;

// "Recording ..." 문자열을 캐싱해두는 전역 변수 및 관련 상수
extern const String recordingStr;          // Recording 문자열 캐시
extern String curAnimatedStr;              // 현재 출력 중인 애니메이션 문자열
extern int recordingIndex;                 // 현재 출력할 문자 인덱스
extern const int RECORDING_DELAY_MS;       // 각 문자가 출력될 주기 (밀리초)

// 함수 선언부
void setScreen(int screen, int durationSec);                 // 화면 전환을 관리하는 함수
void setContentStr(int langCode, String contentStr); // 현재 콘텐츠 설정 함수
void initLCD();                             // LCD 초기화 함수
void clearLCD();                            // LCD 화면을 지우는 함수
void showOpeningScreen();                   // Opening 화면을 보여주는 함수
void drawMicIcon();                         // 마이크 아이콘 화면을 보여주는 함수
void showCurContent();                      // 현재 설정된 콘텐츠를 보여주는 함수
void loopLCD();                             // LCD 루프 제어 함수
void changeUTF(int langCodeInt);            // UTF-8 폰트를 설정하는 함수

#endif // LCD_SCREEN_H
