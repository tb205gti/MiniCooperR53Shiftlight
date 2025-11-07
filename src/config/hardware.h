#pragma once

#include <FastLED.h>
#include <driver/twai.h>
#include <driver/gpio.h>

// LED Pin Definitions
#if CONFIG_IDF_TARGET_ESP32
  #define LED_PIN GPIO_NUM_1      
  #define STATUS_LED GPIO_NUM_8  
#elif CONFIG_IDF_TARGET_ESP32S2
  #define LED_PIN GPIO_NUM_2      
  #define STATUS_LED GPIO_NUM_8  
#elif CONFIG_IDF_TARGET_ESP32S3
  #define LED_PIN GPIO_NUM_2      
  #define STATUS_LED GPIO_NUM_21  
#elif CONFIG_IDF_TARGET_ESP32C3
  #define LED_PIN GPIO_NUM_4      
  #define STATUS_LED GPIO_NUM_8   
#endif

#define NUM_LEDS 8              // Number of LEDs in the strip
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB


#define TWAI_TX_PIN GPIO_NUM_5  // GPIO5 for TWAI TX
#define TWAI_RX_PIN GPIO_NUM_6  // GPIO6 for TWAI RX

#define CAN_RPM_ID 0x316        // Bytes 2-3, 16Bit, Scaling: RPM ≈ raw / 6.4
#define BOOST_CAN_ID 0x565      // Byte 8, bits 0-7, Scaling: ??
#define COOLANT_CAN_ID 0X329    // Byte 2, bit 0-7, 8 bit, Scaling: °C ≈ 0.4865 × raw

#define SERIAL_SPEED 115200


CRGB leds[NUM_LEDS];
CRGB OBLED[1];