#include "config/user.h"
#include "config/hardware.h"

//#include <WiFi.h>
#include <FastLED.h>
#include <driver/twai.h>
#include <driver/gpio.h>

// Timing variables for 10 Hz (100ms interval)
unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 100; // 100ms = 10 Hz

// LED timing variables
const unsigned long blinkInterval = 100; // 100ms for 5Hz blink (on/off) at 7100+ RPM
unsigned long lastStatusBlinkTime = 0;
const unsigned long canBlinkInterval = 500;   // 1Hz (500ms on/off) for CAN error
bool statusBlinkState = false;

// RPM simulation timing
unsigned long lastSimTime = 0;
const unsigned long simInterval = 100; // Update simulation every 100ms (10 Hz)
const unsigned long simPeriod = 10000; // 10-second cycle for RPM simulation

// CAN bus debugging
unsigned long lastDebugPrint = 0;
const unsigned long debugInterval = 1000; // Debug output every 1s
uint32_t rxMsgCount = 0; // Count of received messages
uint32_t errorCount = 0; // Count of CAN errors
bool canConnected = false; // CAN bus connection status

// Logging for speeds
unsigned long lastLogTime = 0;
const unsigned long logInterval = 10000; // 10 seconds
uint32_t canUpdateCount = 0;
uint32_t canUpdateCountPrev = 0;

CRGB leds[NUM_LEDS];
bool ledsChanged = false; // Flag to track if LED state has changed

// CAN signal variables
uint16_t rpm = 0;  // CAN ID 316: Bytes 2-3

// Function prototypes
void updateLEDs();
void updateStatusLEDs();
void simulateRPM();

void setup() {
  Serial.begin(115200);
  
  // Initialize status LED
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  // Initialize FastLED
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);

#ifndef SIMULATE_RPM
  // Initialize TWAI (CAN)
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_PIN, TWAI_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("Failed to install TWAI driver");
    canConnected = false;
    while (1);
  }
  
  if (twai_start() != ESP_OK) {
    Serial.println("Failed to start TWAI");
    canConnected = false;
    while (1);
  }
  Serial.println("TWAI initialized");
  canConnected = true;
#else
  Serial.println("RPM simulation enabled");
#endif

  lastLogTime = millis();
}

void loop() {
  unsigned long currentTime = millis();

#ifdef SIMULATE_RPM
  // Simulate RPM
  if (currentTime - lastSimTime >= simInterval) {
    simulateRPM();
    lastSimTime = currentTime;
  }
#else
  // Check CAN bus status
  twai_status_info_t status;
  twai_get_status_info(&status);
  
  // Check for bus-off state and attempt recovery
  if (status.state == TWAI_STATE_BUS_OFF) {
    if (canConnected) {
      Serial.println("CAN Bus-off detected, attempting recovery");
      canConnected = false;
      twai_initiate_recovery();
    }
  } else if (status.state == TWAI_STATE_STOPPED) {
    if (canConnected) {
      Serial.println("CAN Bus stopped, restarting");
      canConnected = false;
      twai_start();
    }
  } else if (status.state == TWAI_STATE_RUNNING) {
    canConnected = true;
  }

  // Check for TWAI message (non-blocking)
  twai_message_t message;
  esp_err_t result = twai_receive(&message, 0); // 0 ticks for non-blocking
  if (result == ESP_OK) {
    rxMsgCount++;
    switch (message.identifier) {
      case 0x316: // Existing RPM message
        if (message.data_length_code >= 4) {
          uint16_t raw_rpm = (message.data[3] << 8) | message.data[2];
          rpm = raw_rpm / 6.4;
          Serial.printf("CAN Msg: ID=0x%X, DLC=%d, RPM=%d\n",
                        message.identifier, message.data_length_code, rpm);
          canUpdateCount++;
        }
        break;

      default:
        // Ignore other CAN IDs
        break;
    }
  } else if (result != ESP_ERR_TIMEOUT) {
    errorCount++;
    Serial.printf("TWAI receive error: 0x%X\n", result);
  }

  // Periodic debug output
  if (currentTime - lastDebugPrint >= debugInterval) {
    Serial.printf("CAN Status: %s, Rx Count: %lu, Errors: %lu, Bus Errors: %lu, Rx Missed: %lu\n",
                  canConnected ? "Connected" : "Disconnected",
                  rxMsgCount, errorCount, status.bus_error_count, status.rx_missed_count);
    Serial.printf("Signals: RPM=%d\n", rpm);
    lastDebugPrint = currentTime;
  }
#endif

  // Update LEDs at 10 Hz
  if (currentTime - lastUpdateTime >= updateInterval) {
    updateLEDs();
    updateStatusLEDs();
    if (ledsChanged) {
      FastLED.show();
      ledsChanged = false;
    }
    lastUpdateTime = currentTime;
  }

  // Log average speeds every 10 seconds
  if (currentTime - lastLogTime >= logInterval) {
    float elapsed = (float)(currentTime - lastLogTime) / 1000.0;
    float canHz = (float)(canUpdateCount - canUpdateCountPrev) / elapsed;
    Serial.printf("Average CAN update speed: %.2f Hz\n", canHz);
    canUpdateCountPrev = canUpdateCount;
    lastLogTime = currentTime;
  }

  // Yield to FreeRTOS scheduler to reduce CPU load
  vTaskDelay(1 / portTICK_PERIOD_MS);
}

void updateLEDs() {
  static int lastNumPairs = -1;
  static CRGB lastColor = CRGB(0, 0, 0);
  static bool lastRedBlinkState = false;
  bool redBlinkState = false;

  CRGB color;
  int numPairs = constrain(map(rpm, 0, 7100, 0, 4), 0, 4);

  // Determine color based on RPM
  if (rpm < STARTRPM) {
    color = CRGB(0, 0, 0);
  } else if (rpm < MIDRPM) {
    color = CRGB(0, 255, 0);
  } else if (rpm <= MAXRPM) {
    uint8_t t = map(rpm, MIDRPM, MAXRPM, 0, 255);
    color = CRGB(t, 255 - t, 0);
  } else {
    if (millis() - lastUpdateTime >= blinkInterval) {
      redBlinkState = !lastRedBlinkState;
      lastRedBlinkState = redBlinkState;
    }
    color = redBlinkState ? CRGB(255, 0, 0) : CRGB(0, 0, 0);
    numPairs = 4;
  }

  // Only update LEDs if something changed
  if (numPairs != lastNumPairs || color != lastColor || redBlinkState != lastRedBlinkState) {
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB(0, 0, 0);
      if (numPairs >= 1 && (i == 0 || i == 7)) leds[i] = color;
      if (numPairs >= 2 && (i == 1 || i == 6)) leds[i] = color;
      if (numPairs >= 3 && (i == 2 || i == 5)) leds[i] = color;
      if (numPairs >= 4 && (i == 3 || i == 4)) leds[i] = color;
    }
    ledsChanged = true;
    lastNumPairs = numPairs;
    lastColor = color;
  }
}

void updateStatusLEDs() {
  unsigned long currentTime = millis();

  // Status LED on GPIO 21: 
  // - 1Hz blink (500ms on/off) for CAN error only
  if (!canConnected) {
    if (currentTime - lastStatusBlinkTime >= canBlinkInterval) { // 1Hz for CAN error
      statusBlinkState = !statusBlinkState;
      digitalWrite(STATUS_LED, statusBlinkState ? HIGH : LOW);
      lastStatusBlinkTime = currentTime;
    }
  } else {
    digitalWrite(STATUS_LED, LOW); // No errors, LED off
  }
}

void simulateRPM() {
  unsigned long cycleTime = millis() % simPeriod;
  rpm = 1000 + (cycleTime * 8000 / simPeriod); // Linear ramp from 1000 to 9000 RPM
  canUpdateCount++;
}