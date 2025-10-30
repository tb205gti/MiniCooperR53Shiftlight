#pragma once

// LED Pin Definitions
#define LED_PIN GPIO_NUM_4      // GPIO4 for WS2812B data line
#define NUM_LEDS 8              // Number of LEDs in the strip
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
#define STATUS_LED GPIO_NUM_21  // GPIO21 for built-in LED for CAN status

// TWAI pin definitions using gpio_num_t
#define TWAI_TX_PIN GPIO_NUM_5  // GPIO5 for TWAI TX
#define TWAI_RX_PIN GPIO_NUM_6  // GPIO6 for TWAI RX