#pragma once

#if !CONFIG_IDF_TARGET_ESP32S3_SIMULATION
  #define SIMULATE_RPM
#endif

// RPM simulation timing
unsigned long lastSimTime = 0;
const unsigned long simInterval = 100; // Update simulation every 100ms (10 Hz)
const unsigned long simPeriod = 5000; // 10-second cycle for RPM simulation