#pragma once

#define SIMULATE_RPM


// RPM simulation timing
unsigned long lastSimTime = 0;
const unsigned long simInterval = 100; // Update simulation every 100ms (10 Hz)
const unsigned long simPeriod = 5000; // 10-second cycle for RPM simulation