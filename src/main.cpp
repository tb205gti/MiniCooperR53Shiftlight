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
uint8_t receiverMacAddress[] = {0xCC, 0x8D, 0xA2, 0xEC, 0x75, 0xFC};

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
bool ledsChanged = false; // Flag to track if LED state has changed

// CAN signal variables
uint16_t rpm = 0;                // CAN ID 316: Bytes 2-3
uint8_t coolantTemp = 0;         // CAN ID 329: Byte 2, bits 0-7
uint8_t angleFgrPedal = 0;       // CAN ID 329: Byte 5, bits 0-7
uint8_t driverDemand = 0;        // CAN ID 329: Byte 6, bits 0-7
bool checkEngineLight = false;   // CAN ID 545: Byte 1, bit 1
bool engineWarningLight = false; // CAN ID 545: Byte 1, bit 4
bool boostFailureLight = false;  // CAN ID 545: Byte 1, bit 5
bool overheating = false;        // CAN ID 545: Byte 4, bit 3
uint8_t manifoldPressure = 0;    // CAN ID 565: Byte 8, bits 0-7
uint8_t fuelTankLevel = 0;       // CAN ID 613: Byte 3, bits 0-6
bool switchFillingStatus = false;// CAN ID 613: Byte 3, bit 7
bool handbrakeSwitch = false;    // CAN ID 615: Byte 5, bit 1
uint8_t turnSignalIndicator = 0; // CAN ID 615: Byte 6, bits 1-2
bool odbFault = false;           // CAN ID 615: Byte 6, bit 7
uint8_t manualGearSelected = 0;  // CAN ID 618: Byte 2, bits 0-3
bool requestAsc = false;         // CAN ID 153: Byte 1, bit 0
bool requestMsr = false;         // CAN ID 153: Byte 1, bit 1
bool ascLampStatus = false;      // CAN ID 153: Byte 2, bit 0
uint8_t vehicleSpeed = 0;        // CAN ID 153: Byte 2, bits 3-7

// Data structure for ESP-Now transmission
typedef struct {
  uint16_t rpm;
  uint8_t coolantTemp;
  uint8_t angleFgrPedal;
  uint8_t driverDemand;
  uint8_t checkEngineLight : 1;
  uint8_t engineWarningLight : 1;
  uint8_t boostFailureLight : 1;
  uint8_t overheating : 1;
  uint8_t manifoldPressure;
  uint8_t fuelTankLevel;
  uint8_t switchFillingStatus : 1;
  uint8_t handbrakeSwitch : 1;
  uint8_t turnSignalIndicator : 2;
  uint8_t odbFault : 1;
  uint8_t manualGearSelected : 4;
  uint8_t requestAsc : 1;
  uint8_t requestMsr : 1;
  uint8_t ascLampStatus : 1;
  uint8_t vehicleSpeed : 5;
} CanData;

// Instance of the data structure
CanData canData;

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
    switch (message.identifier) {
      case 0x316: // Existing RPM message
        if (message.data_length_code >= 4) {
          uint16_t raw_rpm = (message.data[3] << 8) | message.data[2];
          rpm = raw_rpm / 6.4;
          canData.rpm = rpm;
          Serial.printf("CAN Msg: ID=0x%X, DLC=%d, RPM=%d\n",
                        message.identifier, message.data_length_code, rpm);
        }
        break;

      case 0x329: // Coolant temp, angle FGR pedal, driver demand
        if (message.data_length_code >= 7) {
          coolantTemp = message.data[2]; // Byte 2, bits 0-7
          angleFgrPedal = message.data[5]; // Byte 5, bits 0-7
          driverDemand = message.data[6]; // Byte 6, bits 0-7
          canData.coolantTemp = coolantTemp;
          canData.angleFgrPedal = angleFgrPedal;
          canData.driverDemand = driverDemand;
          Serial.printf("CAN Msg: ID=0x%X, DLC=%d, CoolantTemp=%d, AngleFgrPedal=%d, DriverDemand=%d\n",
                        message.identifier, message.data_length_code, coolantTemp, angleFgrPedal, driverDemand);
        }
        break;

      case 0x545: // Check engine light, engine warning light, boost failure light, overheating
        if (message.data_length_code >= 5) {
          checkEngineLight = (message.data[1] >> 1) & 0x01; // Byte 1, bit 1
          engineWarningLight = (message.data[1] >> 4) & 0x01; // Byte 1, bit 4
          boostFailureLight = (message.data[1] >> 5) & 0x01; // Byte 1, bit 5
          overheating = (message.data[4] >> 3) & 0x01; // Byte 4, bit 3
          canData.checkEngineLight = checkEngineLight;
          canData.engineWarningLight = engineWarningLight;
          canData.boostFailureLight = boostFailureLight;
          canData.overheating = overheating;
          Serial.printf("CAN Msg: ID=0x%X, DLC=%d, CheckEngine=%d, EngWarning=%d, BoostFail=%d, Overheating=%d\n",
                        message.identifier, message.data_length_code, checkEngineLight,
                        engineWarningLight, boostFailureLight, overheating);
        }
        break;

      case 0x565: // Manifold absolute pressure
        if (message.data_length_code >= 8) {
          manifoldPressure = message.data[7]; // Byte 8, bits 0-7 (byte indexing starts at 0)
          canData.manifoldPressure = manifoldPressure;
          Serial.printf("CAN Msg: ID=0x%X, DLC=%d, ManifoldPressure=%d\n",
                        message.identifier, message.data_length_code, manifoldPressure);
        }
        break;

      case 0x613: // Fuel tank level, switch filling status
        if (message.data_length_code >= 4) {
          fuelTankLevel = message.data[3] & 0x7F; // Byte 3, bits 0-6
          switchFillingStatus = (message.data[3] >> 7) & 0x01; // Byte 3, bit 7
          canData.fuelTankLevel = fuelTankLevel;
          canData.switchFillingStatus = switchFillingStatus;
          Serial.printf("CAN Msg: ID=0x%X, DLC=%d, FuelTankLevel=%d, SwitchFilling=%d\n",
                        message.identifier, message.data_length_code, fuelTankLevel, switchFillingStatus);
        }
        break;

      case 0x615: // Handbrake switch, turn signal indicator, OBD fault
        if (message.data_length_code >= 7) {
          handbrakeSwitch = (message.data[5] >> 1) & 0x01; // Byte 5, bit 1
          turnSignalIndicator = (message.data[6] >> 1) & 0x03; // Byte 6, bits 1-2
          odbFault = (message.data[6] >> 7) & 0x01; // Byte 6, bit 7
          canData.handbrakeSwitch = handbrakeSwitch;
          canData.turnSignalIndicator = turnSignalIndicator;
          canData.odbFault = odbFault;
          Serial.printf("CAN Msg: ID=0x%X, DLC=%d, Handbrake=%d, TurnSignal=%d, ODBFault=%d\n",
                        message.identifier, message.data_length_code, handbrakeSwitch,
                        turnSignalIndicator, odbFault);
        }
        break;

      case 0x618: // Manual gear selected
        if (message.data_length_code >= 3) {
          manualGearSelected = message.data[2] & 0x0F; // Byte 2, bits 0-3
          canData.manualGearSelected = manualGearSelected;
          Serial.printf("CAN Msg: ID=0x%X, DLC=%d, ManualGear=%d\n",
                        message.identifier, message.data_length_code, manualGearSelected);
        }
        break;

      case 0x153: // Request ASC, request MSR, ASC lamp status, vehicle speed
        if (message.data_length_code >= 3) {
          requestAsc = (message.data[1] >> 0) & 0x01; // Byte 1, bit 0
          requestMsr = (message.data[1] >> 1) & 0x01; // Byte 1, bit 1
          ascLampStatus = (message.data[2] >> 0) & 0x01; // Byte 2, bit 0
          vehicleSpeed = (message.data[2] >> 3) & 0x1F; // Byte 2, bits 3-7
          canData.requestAsc = requestAsc;
          canData.requestMsr = requestMsr;
          canData.ascLampStatus = ascLampStatus;
          canData.vehicleSpeed = vehicleSpeed;
          Serial.printf("CAN Msg: ID=0x%X, DLC=%d, ASCReq=%d, MSRReq=%d, ASCLamp=%d, VehicleSpeed=%d\n",
                        message.identifier, message.data_length_code, requestAsc, requestMsr,
                        ascLampStatus, vehicleSpeed);
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
    Serial.printf("Signals: RPM=%d, CoolantTemp=%d, AngleFgrPedal=%d, DriverDemand=%d, "
                  "CheckEngine=%d, EngWarning=%d, BoostFail=%d, Overheating=%d, ManifoldPressure=%d, "
                  "FuelTankLevel=%d, SwitchFilling=%d, Handbrake=%d, TurnSignal=%d, "
                  "ODBFault=%d, ManualGear=%d, ASCReq=%d, MSRReq=%d, ASCLamp=%d, VehicleSpeed=%d\n",
                  rpm, coolantTemp, angleFgrPedal, driverDemand, checkEngineLight, engineWarningLight,
                  boostFailureLight, overheating, manifoldPressure, fuelTankLevel,
                  switchFillingStatus, handbrakeSwitch, turnSignalIndicator, odbFault,
                  manualGearSelected, requestAsc, requestMsr, ascLampStatus, vehicleSpeed);
    lastDebugPrint = currentTime;
  }
#endif

  // Send all CAN data via ESP-Now at 10 Hz
  if (currentTime - lastSendTime >= sendInterval) {
    esp_err_t result = esp_now_send(receiverMacAddress, (uint8_t *)&canData, sizeof(CanData));
    if (result != ESP_OK) {
      Serial.println("Error sending CAN data");
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
  // Serial.print("Last Packet Send Status: ");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
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
  canData.rpm = rpm;
  Serial.print("Simulated RPM: ");
  Serial.println(rpm);
}