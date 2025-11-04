#pragma once

#define STALLED 500
#define LOWRPM 3500
#define MIDRPM 5000
#define HIGHRPM 7100


unsigned char brightness = 25;
const unsigned long canBlinkInterval = 500;   // 1Hz (500ms on/off) for CAN error
const unsigned long blinkInterval = 100; // 100ms for 5Hz blink (on/off) at 7100+ RPM
const unsigned long updateInterval = 100; // 100ms = 10 Hz
const unsigned long debugInterval = 1000; // Debug output every 1s
const unsigned long logInterval = 10000; // 10 seconds


static CRGB bootColor = CRGB(255, 29, 0);
static CRGB BLACK = CRGB(0, 0, 0);
static CRGB LOW_COLOR = CRGB(0, 0, 255); //255,0,0
static CRGB MID_COLOR = CRGB(0, 255, 0); //0,255,0
static CRGB HIGH_COLOR = CRGB(255, 0, 0);