//#include <WiFi.h>
#include "config/hardware.h"
#include "config/user.h"
#include "config/simulation.h"

// Timing variables for 10 Hz (100ms interval)
unsigned long lastUpdateTime = 0;
unsigned long lastStatusBlinkTime = 0;
bool statusBlinkState = false;
unsigned long lastDebugPrint = 0;
uint32_t rxMsgCount = 0; // Count of received messages
uint32_t errorCount = 0; // Count of CAN errors
bool canConnected = false; // CAN bus connection status

// Logging for speeds
unsigned long lastLogTime = 0;
uint32_t canUpdateCount = 0;
uint32_t canUpdateCountPrev = 0;
bool ledsChanged = false; // Flag to track if LED state has changed
uint16_t rpm = 0;        

bool faderdone = false;
bool boostGauge = true;
bool shiftLight = false;
unsigned char boost = 0;
bool goingup = true;

// Function prototypes
void updateLEDs();
void updateStatusLEDs();
void simulateRPM();
void bootAnimation();
void showBoost();

void bootAnimation(){
  static uint32_t animatedelay = 60;
  static uint32_t brightdelay = 20;

  // Make a default startup sequence
  for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = bootColor;
        FastLED.show();
        delay(animatedelay);
      }
  delay(150);

  for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = BLACK;
        FastLED.show();
        delay(animatedelay);
      }

  delay(100);

  for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = bootColor;
      }

  for (int i = 0; i<20; i++){
    FastLED.setBrightness(i);
    FastLED.show();
    delay(brightdelay);
  }

  for (int i = 20; i>=0; i--){
    FastLED.setBrightness(i);
    FastLED.show();
    delay(brightdelay);
  }
  delay(500);
}

void setup() {
  Serial.begin(SERIAL_SPEED);

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);

#if CONFIG_IDF_TARGET_ESP32S3
  OBLED[0] = bootColor;
  FastLED.addLeds<WS2812, STATUS_LED, RGB>(OBLED, 1); //Only for S3 variant!
#elif !CONFIG_IDF_TARGET_ESP32c3
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH);
#else
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
#endif

  FastLED.setBrightness(20);
  bootAnimation();

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
      case CAN_RPM_ID: // Existing RPM message
        if (message.data_length_code >= 4) {
          uint16_t raw_rpm = (message.data[3] << 8) | message.data[2];
          rpm = raw_rpm / 6.4;
          Serial.printf("CAN Msg: ID=0x%X, DLC=%d, RPM=%d\n",
                        message.identifier, message.data_length_code, rpm);
          canUpdateCount++;
        }
        break;

      case BOOST_CAN_ID:
        //Do whatever we need with the boost data..
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

    if (!faderdone){
      if (fadebright >= brightness){
        //FastLED.setBrightness(brightness);  
        faderdone = true;
      }
      else{
        FastLED.setBrightness(fadebright);
        FastLED.show();
        delay(50);
        fadebright += 2;
      }
    }

  // Update LEDs at updateInterval 
  if (currentTime - lastUpdateTime >= updateInterval) {
    if (shiftLight)
     updateLEDs();
    else if (boostGauge)
      showBoost();

    updateStatusLEDs();
    if (ledsChanged) {
      FastLED.show();
      ledsChanged = false;
    }
    lastUpdateTime = currentTime;
  }

  // Log average speeds every 10 seconds
  if ((Serial) && currentTime - lastLogTime >= logInterval) {
    float elapsed = (float)(currentTime - lastLogTime) / 1000.0;
    float canHz = (float)(canUpdateCount - canUpdateCountPrev) / elapsed;
    Serial.printf("Average CAN update speed: %.2f Hz\n", canHz);
    Serial.println("Rpm: "+ (String) rpm);
    canUpdateCountPrev = canUpdateCount;
    lastLogTime = currentTime;
    shiftLight = !shiftLight;
    boostGauge = !boostGauge;
  }

  // Yield to FreeRTOS scheduler to reduce CPU load
  vTaskDelay(1 / portTICK_PERIOD_MS);
}

void showBoost(){
  ledsChanged = false;
  goingup? boost++:boost--;

  if (boost >= 18){
    goingup = false;
  }
  else if (boost <= 0)
    goingup = true;

  unsigned char numLeds = (unsigned char) boost / 2;
  unsigned char dim = boost % 2;

  leds[0] = BOOST_LOW;
  leds[1] = BOOST_LOW;
  leds[2] = BOOST_LOW;
  leds[3] = BOOST_MID;
  leds[4] = BOOST_MID;
  leds[5] = BOOST_HIGH;
  leds[6] = BOOST_HIGH;
  leds[7] = BOOST_TOP;

  for (int i = 0; i < NUM_LEDS; i++) {
    if (i == numLeds && dim){
      leds[i].nscale8(20);
    }
    else if (i > numLeds-1)
      leds[i] = BLACK;
  }


  ledsChanged = true;
}

void updateLEDs() {
  static int lastNumPairs = -1;
  static CRGB lastColor = BLACK;
  static bool lastRedBlinkState = false;
  bool redBlinkState = false;

  CRGB color;
  int numPairs = constrain(map(rpm, 0, HIGHRPM, 0, 4), 0, 4);
  blinkInterval = constrain(blinkInterval,0,updateInterval);
  // Determine color based on RPM
  if (rpm < STALLED)
    numPairs = 2;
  else if (rpm < LOWRPM)
   color = BLACK;
  else if (rpm < MIDRPM) {
    color = MID_COLOR;
  } else if (rpm <= HIGHRPM) {
    uint8_t t = map(rpm, MIDRPM, HIGHRPM, 0, 255);
    color = CRGB(t, 255 - t, 0);
  } else {
    if (millis() - lastUpdateTime >= blinkInterval) {
      redBlinkState = !lastRedBlinkState;
      lastRedBlinkState = redBlinkState;
    }
    color = redBlinkState ? HIGH_COLOR : BLACK;
    numPairs = 4;
  }

  // Only update LEDs if something changed
  if (numPairs != lastNumPairs || color != lastColor || redBlinkState != lastRedBlinkState) {
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB(0, 0, 0);
      if (numPairs >= 1 && (i == 0 || i == 7)) leds[i] = color;
      if (numPairs >= 2 && (i == 1 || i == 6)) leds[i] = color;
      if (rpm < STALLED && (numPairs >= 2 && (i == 3 || i == 4))) leds[i] = STALL_COLOR; //light two centers if engine has stalled
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

  // Status LED on GPIO 8: 
  // - 1Hz blink (500ms on/off) for CAN error only
  if (!canConnected) {
    if (currentTime - lastStatusBlinkTime >= canBlinkInterval) { // 1Hz for CAN error
      statusBlinkState = !statusBlinkState;
      #if CONFIG_IDF_TARGET_ESP32S3
        OBLED[0] = statusBlinkState ? bootColor: BLACK;
      #else
        digitalWrite(STATUS_LED, statusBlinkState ? HIGH : LOW);
      #endif
      lastStatusBlinkTime = currentTime;
    }
  } else {
    #if !CONFIG_IDF_TARGET_ESP32c3
      digitalWrite(STATUS_LED, LOW); // No errors, LED off (ESP32C3 has reversed LED logic, HIGH is off..)
    #elif CONFIG_IDF_TARGET_ESP32S3
      OBLED[0] = BLACK;
    #else
      digitalWrite(STATUS_LED, HIGH); 
    #endif
  }
}

void simulateRPM() {
  unsigned long cycleTime = millis() % simPeriod;
  rpm = 1000 + (cycleTime * 12000 / simPeriod); // Linear ramp from 1000 to 9000 RPM
  canUpdateCount++;
}