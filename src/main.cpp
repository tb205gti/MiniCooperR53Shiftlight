#include <WiFi.h>
#include <FastLED.h>
#include <driver/twai.h>
#include <driver/gpio.h>
#include <esp_now.h>
#include <esp_wifi.h>

// Simulator option: Uncomment to simulate RPM, comment to use CAN bus
//#define SIMULATE_RPM

// Wi-Fi channel for ESP-Now
#define WIFI_CHANNEL 1

// Receiver ESP32 MAC Address (replace with actual MAC address)
uint8_t receiverMacAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// LED Pin Definitions
#define LED_PIN GPIO_NUM_4      // GPIO4 for WS2812B data line
#define NUM_LEDS 8              // Number of LEDs in the strip
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
#define STATUS_LED GPIO_NUM_21  // GPIO21 for built-in LED for both ESP-Now and CAN status

// TWAI pin definitions using gpio_num_t
#define TWAI_TX_PIN GPIO_NUM_5  // GPIO5 for TWAI TX
#define TWAI_RX_PIN GPIO_NUM_6  // GPIO6 for TWAI RX

// Timing variables for 10 Hz (100ms interval)
unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 100; // 100ms = 10 Hz
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 100; // 100ms = 10 Hz for ESP-Now

// LED timing variables
const unsigned long blinkInterval = 100; // 100ms for 5Hz blink (on/off) at 7100+ RPM
unsigned long lastStatusBlinkTime = 0;
const unsigned long espNowBlinkInterval = 200; // 2.5Hz (200ms on/off) for ESP-Now error
const unsigned long canBlinkInterval = 500;   // 1Hz (500ms on/off) for CAN error
bool statusBlinkState = false;

// RPM simulation timing
unsigned long lastSimTime = 0;
const unsigned long simInterval = 100; // Update simulation every 100ms
const unsigned long simPeriod = 20000; // 20-second cycle for RPM simulation

// CAN bus debugging
unsigned long lastDebugPrint = 0;
const unsigned long debugInterval = 1000; // Debug output every 1s
uint32_t rxMsgCount = 0; // Count of received messages
uint32_t errorCount = 0; // Count of CAN errors
bool canConnected = false; // CAN bus connection status
bool espNowError = false; // ESP-Now send error status

CRGB leds[NUM_LEDS];
uint16_t rpm = 0; // Current RPM value
bool ledsChanged = false; // Flag to track if LED state has changed

// Function prototypes
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
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
  FastLED.setBrightness(75);

  // Initialize Wi-Fi in station mode
  WiFi.mode(WIFI_STA);
  if (esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    Serial.println("Failed to set Wi-Fi channel");
    espNowError = true;
    while (1);
  }

  // Initialize ESP-Now
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-Now");
    espNowError = true;
    while (1);
  }

  // Register send callback
  esp_now_register_send_cb(OnDataSent);

  // Add peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    espNowError = true;
    while (1);
  }
  Serial.println("ESP-Now peer added");

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

  Serial.print("Sender MAC Address: ");
  Serial.println(WiFi.macAddress());
  Serial.print("Wi-Fi Channel: ");
  Serial.println(WiFi.channel());
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
    if (message.identifier == 0x316 && message.data_length_code >= 4) {
      uint16_t raw_rpm = (message.data[3] << 8) | message.data[2];
      rpm = raw_rpm / 6.4;
      Serial.printf("CAN Msg: ID=0x%X, DLC=%d, Data=[%02X %02X %02X %02X], RPM=%d\n",
                    message.identifier, message.data_length_code,
                    message.data[0], message.data[1], message.data[2], message.data[3], rpm);
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
    lastDebugPrint = currentTime;
  }
#endif

  // Send RPM via ESP-Now at 10 Hz
  if (currentTime - lastSendTime >= sendInterval) {
    esp_err_t result = esp_now_send(receiverMacAddress, (uint8_t *)&rpm, sizeof(rpm));
    if (result != ESP_OK) {
      Serial.println("Error sending RPM data");
      espNowError = true;
    } else {
      espNowError = false; // Clear error on successful send
    }
    lastSendTime = currentTime;
  }

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
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  espNowError = (status != ESP_NOW_SEND_SUCCESS);
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void updateLEDs() {
  static int lastNumPairs = -1;
  static CRGB lastColor = CRGB(0, 0, 0);
  static bool lastRedBlinkState = false;
  bool redBlinkState = false;

  CRGB color;
  int numPairs = constrain(map(rpm, 0, 7100, 0, 4), 0, 4);

  // Determine color based on RPM
  if (rpm < 3000) {
    color = CRGB(0, 0, 0);
  } else if (rpm < 6000) {
    color = CRGB(0, 255, 0);
  } else if (rpm <= 7100) {
    uint8_t t = map(rpm, 6000, 7100, 0, 255);
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
  // - 2.5Hz blink (200ms on/off) for ESP-Now error only
  // - 1Hz blink (500ms on/off) for CAN error only
  // - 0.5Hz blink (1000ms on/off) for both ESP-Now and CAN errors
  if (espNowError && !canConnected) {
    if (currentTime - lastStatusBlinkTime >= 1000) { // 0.5Hz for both errors
      statusBlinkState = !statusBlinkState;
      digitalWrite(STATUS_LED, statusBlinkState ? HIGH : LOW);
      lastStatusBlinkTime = currentTime;
    }
  } else if (espNowError) {
    if (currentTime - lastStatusBlinkTime >= espNowBlinkInterval) { // 2.5Hz for ESP-Now error
      statusBlinkState = !statusBlinkState;
      digitalWrite(STATUS_LED, statusBlinkState ? HIGH : LOW);
      lastStatusBlinkTime = currentTime;
    }
  } else if (!canConnected) {
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
  float cycleTime = (millis() % simPeriod) / 1000.0;
  if (cycleTime <= 10.0) {
    rpm = 1000 + (uint16_t)(8000 * cycleTime / 10.0);
  } else {
    rpm = 9000 - (uint16_t)(8000 * (cycleTime - 10.0) / 10.0);
  }
  Serial.print("Simulated RPM: ");
  Serial.println(rpm);
}