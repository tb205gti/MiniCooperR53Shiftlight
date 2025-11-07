#pragma once

#define STALLED 500
#define LOWRPM 3000
#define MIDRPM 5000
#define HIGHRPM 7100

unsigned char brightness = 65;
unsigned char fadebright = 0;
const unsigned long canBlinkInterval = 500;   // 1Hz (500ms on/off) for CAN error
unsigned long blinkInterval = 75; // 100ms for 5Hz blink (on/off) at HIGHRPM+ RPM
unsigned long updateInterval = 75; // 100ms = 10 Hz
const unsigned long debugInterval = 1000; // Debug output every 1s
const unsigned long logInterval = 10000; // 10 seconds

static CRGB bootColor = CRGB(255, 29, 0);
//static CRGB bootColor = CRGB(246, 0, 255);
static CRGB BLACK = CRGB(0, 0, 0);
static CRGB STALL_COLOR = CRGB(155, 29, 0);
static CRGB LOW_COLOR = CRGB(255, 229, 0); //0 29 0
static CRGB MID_COLOR = CRGB(0, 255, 0); 
static CRGB HIGH_COLOR = CRGB(255, 0, 0);

// static CRGB BOOST_LOW = CRGB(0, 255, 0);
// static CRGB BOOST_MID = CRGB(255, 229, 0);
// static CRGB BOOST_HIGH = CRGB(155, 29, 0);
// static CRGB BOOST_TOP = CRGB(255, 0, 0);

static CRGB BOOST_LOW = CRGB(255, 29, 0);
static CRGB BOOST_MID = CRGB(255, 29, 0);
static CRGB BOOST_HIGH = CRGB(255, 29, 0);
static CRGB BOOST_TOP = CRGB(255, 29, 0);