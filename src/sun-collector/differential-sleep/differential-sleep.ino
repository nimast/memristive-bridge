// Memristive bridge sender
// Copyright (c) Nimrod Astarhan 2025
// This sketch needs v3 of esp board lib to work with ESP32H2
// Last used 3.1.3

// Make sure these are defined as actual integer values, not TRUE/FALSE
#define DEBUG 0  // Set to 1 for detailed debug output, 0 for minimal output
#define INFO 1   // Set to 1 for important info messages, 0 for silent operation
#define DISABLE_SLEEP 1  // Set to 1 to disable sleep for debugging

#include <Adafruit_ADS1X15.h>
#include <Adafruit_NeoPixel.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>
#include <esp_pm.h>
#include <driver/gpio.h>

// Updated I2C pins for ESP32-H2-DEV-KIT-N4
#define I2C_SDA 4  // GPIO4 for SDA
#define I2C_SCL 5  // GPIO5 for SCL
// I2C pins for ESP32-S3
// #define I2C_SDA 8  
// #define I2C_SCL 9  
// RGB LED pin for ESP32-S3
// #define PIN 21
// RGB LED pins for ESP32-H2-DEV-KIT-N4
#define LED_R GPIO_NUM_2  // GPIO2 for Red
#define LED_G GPIO_NUM_3  // GPIO3 for Green
#define LED_B GPIO_NUM_4  // GPIO4 for Blue
#define DELAYVAL 500
#define LED_ON_TIME 5000  // 5 seconds in milliseconds

// BLE settings
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define DEVICE_NAME         "MemristiveBridge"
#define BLE_CONNECTION_TIMEOUT 30000  // 30 seconds max to wait for connection (increased from 10s)
#define BLE_RECONNECT_ATTEMPTS 5      // Number of times to try reconnecting (increased from 3)
#define BLE_ADVERTISE_PERIOD 10000     // milliseconds of advertising before checking connection 
#define BLE_CONNECTION_WAIT_TIME 5000  // milliseconds to wait for connection to complete after advertising is seen
#define BLE_BEACON_DURATION 10000  // Advertise for 10 seconds when change detected

// Power saving settings
#define SLEEP_DURATION 5 // Sleep duration in seconds
#define HEARTBEAT_INTERVAL 5000 // Heartbeat interval in milliseconds

// Change detection parameters
#define WINDOW_SIZE 5                 // Smaller window size for faster response
#define MIN_THRESHOLD 0.1000          // Minimum threshold set to exactly one bit of the ADS1115 (in mV)
#define MAX_THRESHOLD 1.0            // Lower maximum threshold (in mV)
#define ADAPTIVE_FACTOR 1.001           // Even gentler adaptation factor
#define STABILITY_THRESHOLD 0.1       // Lower threshold to determine if signal is stable (in mV)
#define MIN_TRIGGER_INTERVAL 5000     // Reduced from 60000 to 5000 (5 seconds between triggers)
#define HOUR_MS 3600000               // One hour in milliseconds
#define MAX_TRIGGERS_PER_HOUR 30      // Maximum number of triggers per hour

// BLE objects
BLEServer* pServer = nullptr;
BLECharacteristic* pCharacteristic = nullptr;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// Debug and info print functions - simplified for reliability
void debugPrintln(const char* message) {
  if (DEBUG) {
    Serial.println(message);
    Serial.flush();
  }
}

void debugPrint(const char* message) {
  if (DEBUG) {
    Serial.print(message);
  }
}

void infoPrintln(const char* message) {
  if (INFO) {
    Serial.println(message);
    Serial.flush();
  }
}

void infoPrint(const char* message) {
  if (INFO) {
    Serial.print(message);
  }
}

void debugPrintln(float value, int precision = 2) {
  if (DEBUG) {
    Serial.println(value, precision);
    Serial.flush();
  }
}

void debugPrint(float value, int precision = 2) {
  if (DEBUG) {
    Serial.print(value, precision);
  }
}

void infoPrintln(float value, int precision = 2) {
  if (INFO) {
    Serial.println(value, precision);
    Serial.flush();
  }
}

void infoPrint(float value, int precision = 2) {
  if (INFO) {
    Serial.print(value, precision);
  }
}

void debugPrintln(int value) {
  if (DEBUG) {
    Serial.println(value);
    Serial.flush();
  }
}

void debugPrint(int value) {
  if (DEBUG) {
    Serial.print(value);
  }
}

void infoPrintln(int value) {
  if (INFO) {
    Serial.println(value);
    Serial.flush();
  }
}

void infoPrint(int value) {
  if (INFO) {
    Serial.print(value);
  }
}

void debugPrintln(unsigned long value) {
  if (DEBUG) {
    Serial.println(value);
    Serial.flush();
  }
}

void debugPrint(unsigned long value) {
  if (DEBUG) {
    Serial.print(value);
  }
}

void infoPrintln(unsigned long value) {
  if (INFO) {
    Serial.println(value);
    Serial.flush();
  }
}

void infoPrint(unsigned long value) {
  if (INFO) {
    Serial.print(value);
  }
}

// Add debug and info functions for flushing and delay when needed
void debugFlushAndDelay(int delayMs = 100) {
  if (DEBUG) {
    Serial.flush();
    if (delayMs > 0) {
      delay(delayMs);
    }
  }
}

void infoFlushAndDelay(int delayMs = 100) {
  if (INFO) {
    Serial.flush();
    if (delayMs > 0) {
      delay(delayMs);
    }
  }
}

// Callback class for BLE server
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      infoPrintln("Device connected");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      infoPrintln("Device disconnected");
    }
};

// Variables for change detection
float values[WINDOW_SIZE];            // Circular buffer for recent values
int valueIndex = 0;                   // Current index in the circular buffer
float currentThreshold = MIN_THRESHOLD; // Current adaptive threshold
unsigned long lastTriggerTime = 0;    // Time of the last trigger
unsigned long hourStartTime = 0;      // Start time of the current hour
int triggerCount = 0;                 // Number of triggers in the current hour
float lastAverage = 0;                // Last calculated average
bool bufferFilled = false;            // Flag to indicate if the buffer is filled
float baselineValue = 0;              // Baseline value for comparison
bool baselineEstablished = false;     // Flag to indicate if baseline is established
int stableReadingsCount = 0;          // Count of consecutive stable readings
unsigned long lastHeartbeatTime = 0;  // Time of the last heartbeat

// RTC memory to preserve state during sleep
RTC_DATA_ATTR int rtc_valueIndex = 0;
RTC_DATA_ATTR float rtc_values[WINDOW_SIZE];
RTC_DATA_ATTR float rtc_currentThreshold = MIN_THRESHOLD;
RTC_DATA_ATTR unsigned long rtc_lastTriggerTime = 0;
RTC_DATA_ATTR unsigned long rtc_hourStartTime = 0;
RTC_DATA_ATTR int rtc_triggerCount = 0;
RTC_DATA_ATTR float rtc_lastAverage = 0;
RTC_DATA_ATTR bool rtc_bufferFilled = false;
RTC_DATA_ATTR float rtc_baselineValue = 0;
RTC_DATA_ATTR bool rtc_baselineEstablished = false;
RTC_DATA_ATTR int rtc_stableReadingsCount = 0;
RTC_DATA_ATTR bool rtc_firstBoot = true;

// Global objects
TwoWire s3i2c(0);  // This constructor already initializes I2C
Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */

#define RGB_BRIGHTNESS 10  // Brightness level for RGB LED

// LED control functions
void setLEDColor(uint8_t r, uint8_t g, uint8_t b) {
  rgbLedWrite(RGB_BUILTIN, g, r, b);  // Swapped r and g for GRB order
}

// Initialize BLE as a beacon
void initBLE() {
  // Create the BLE Device
  BLEDevice::init(DEVICE_NAME);

  // Set maximum transmit power
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
  infoPrintln("BLE device initialized with maximum power");

  // Configure advertising with maximum visibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  
  // Prepare advertisement data with explicit flags and service UUID
  BLEAdvertisementData advData;
  advData.setFlags(0x06); // BR_EDR_NOT_SUPPORTED | LE General Discoverable Mode
  advData.setCompleteServices(BLEUUID(SERVICE_UUID));
  advData.setName(DEVICE_NAME);
  
  pAdvertising->setAdvertisementData(advData);
  
  // Set scan response data explicitly
  BLEAdvertisementData scanResponse;
  scanResponse.setName(DEVICE_NAME);
  scanResponse.setCompleteServices(BLEUUID(SERVICE_UUID));
  pAdvertising->setScanResponseData(scanResponse);
  
  // Configure advertising parameters for better visibility
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // Helps with iPhone connections
  pAdvertising->setMaxPreferred(0x12);  // Increased visibility
  
  infoPrintln("BLE advertising configured for maximum visibility");
  infoPrintln("BLE initialization complete");
}

// Deinitialize BLE to save power
void deinitBLE() {
  if (pServer != nullptr) {
    pServer->getAdvertising()->stop();
    BLEDevice::deinit();
    pServer = nullptr;
    pCharacteristic = nullptr;
  }
}

// Simple beacon advertising function - doesn't wait for a connection
void advertiseBriefly() {
  // Don't need to create a server with a characteristic anymore
  
  // Configure advertising for better discoverability
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  
  // Prepare advertisement data with explicit flags and service UUID
  BLEAdvertisementData advData;
  advData.setFlags(0x06); // BR_EDR_NOT_SUPPORTED | LE General Discoverable Mode
  advData.setCompleteServices(BLEUUID(SERVICE_UUID));
  advData.setName(DEVICE_NAME);
  
  pAdvertising->setAdvertisementData(advData);
  
  // Set scan response data explicitly
  BLEAdvertisementData scanResponse;
  scanResponse.setName(DEVICE_NAME);
  scanResponse.setCompleteServices(BLEUUID(SERVICE_UUID));
  pAdvertising->setScanResponseData(scanResponse);
  
  // Configure advertising parameters for better visibility
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // Helps with iPhone connections
  pAdvertising->setMaxPreferred(0x12);  // Increased visibility
  
  // Set max TX power for maximum range
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
  
  // Start advertising
  infoPrintln("Started beacon advertising");
  pAdvertising->start();
  
  // Visual indication - Blue for advertising
  setLEDColor(0, 0, RGB_BRIGHTNESS);
  
  // Advertise for the beacon duration
  unsigned long startTime = millis();
  while (millis() - startTime < BLE_BEACON_DURATION) {
    // Blink LED to show we're advertising
    if ((millis() / 250) % 2 == 0) {
      setLEDColor(0, 0, RGB_BRIGHTNESS);  // Full brightness blue
    } else {
      setLEDColor(0, 0, RGB_BRIGHTNESS/3);  // Dimmer blue
    }
    
    // Print status occasionally
    if (millis() % 2000 < 10) {
      infoPrint("Beacon advertising... ");
      infoPrint((millis() - startTime) / 1000);
      infoPrint(" of ");
      infoPrintln(BLE_BEACON_DURATION / 1000);
    }
    
    delay(50);  // Short delay to prevent CPU hogging
  }
  
  // Stop advertising
  pAdvertising->stop();
  infoPrintln("Beacon advertising completed");
  
  // Turn off LED
  setLEDColor(0, 0, 0);
}

// Save state to RTC memory before sleep
void saveStateToRTC() {
  rtc_valueIndex = valueIndex;
  for (int i = 0; i < WINDOW_SIZE; i++) {
    rtc_values[i] = values[i];
  }
  rtc_currentThreshold = currentThreshold;
  rtc_lastTriggerTime = lastTriggerTime;
  rtc_hourStartTime = hourStartTime;
  rtc_triggerCount = triggerCount;
  rtc_lastAverage = lastAverage;
  rtc_bufferFilled = bufferFilled;
  rtc_baselineValue = baselineValue;
  rtc_baselineEstablished = baselineEstablished;
  rtc_stableReadingsCount = stableReadingsCount;
  rtc_firstBoot = false;
}

// Restore state from RTC memory after wake up
void restoreStateFromRTC() {
  if (rtc_firstBoot) return;
  
  valueIndex = rtc_valueIndex;
  for (int i = 0; i < WINDOW_SIZE; i++) {
    values[i] = rtc_values[i];
  }
  currentThreshold = rtc_currentThreshold;
  
  // Adjust time-based variables for the sleep period
  unsigned long sleepTimeMs = SLEEP_DURATION * 1000;
  lastTriggerTime = rtc_lastTriggerTime + sleepTimeMs;
  hourStartTime = rtc_hourStartTime + sleepTimeMs;
  
  triggerCount = rtc_triggerCount;
  lastAverage = rtc_lastAverage;
  bufferFilled = rtc_bufferFilled;
  baselineValue = rtc_baselineValue;
  baselineEstablished = rtc_baselineEstablished;
  stableReadingsCount = rtc_stableReadingsCount;
  
  infoPrintln("State restored from RTC memory");
}

void setup()
{
  // Initialize serial first, before anything else
  Serial.begin(115200);  // Use explicit baud rate
  delay(2000);  // Give serial time to initialize
  
  Serial.println("\n\n\nMemristive Bridge Initializing...");
  Serial.println("DEBUG is ON");  // Explicitly show debug status
  Serial.println("INFO is ON");   // Explicitly show info status
  
  // Test LED
  infoPrintln("Testing LED...");
  setLEDColor(RGB_BRIGHTNESS, 0, 0);  // Red
  delay(1000);
  setLEDColor(0, RGB_BRIGHTNESS, 0);  // Green
  delay(1000);
  setLEDColor(0, 0, RGB_BRIGHTNESS);  // Blue
  delay(1000);
  setLEDColor(0, 0, 0);  // Off
  infoPrintln("LED test complete");

  // Initialize BLE
  infoPrintln("Initializing BLE...");
  initBLE();
  infoPrintln("BLE initialization complete");

  // Restore state if not first boot
  infoPrintln("Restoring state from RTC...");
  restoreStateFromRTC();
  infoPrintln("State restoration complete");

  // Initialize I2C and ADS
  infoPrintln("Initializing I2C and ADS...");
  
  // First, end any existing I2C connection
  s3i2c.end();
  delay(100);  // Give the bus time to settle
  
  // Initialize I2C with explicit pins
  if (!s3i2c.begin(I2C_SDA, I2C_SCL, 100000)) {
    infoPrintln("Failed to initialize I2C!");
    while (1) {
      setLEDColor(0, RGB_BRIGHTNESS, 0);  // Green to indicate error
      delay(500);
      setLEDColor(0, 0, 0);
      delay(500);
    }
  }
  infoPrintln("I2C initialized successfully");

  // Scan I2C bus for devices
  infoPrintln("Scanning I2C bus...");
  for (byte addr = 1; addr < 127; addr++) {
    s3i2c.beginTransmission(addr);
    byte error = s3i2c.endTransmission();
    if (error == 0) {
      infoPrint("I2C device found at address 0x");
      if (addr < 16) infoPrint("0");
      infoPrintln(addr, HEX);
    }
  }

  // Initialize ADS
  infoPrintln("Initializing ADS...");
  if (!ads.begin(ADS1X15_ADDRESS, &s3i2c)) {
    infoPrintln("Failed to initialize ADS!");
    while (1) {
      setLEDColor(RGB_BRIGHTNESS, 0, 0);  // Red to indicate error
      delay(500);
      setLEDColor(0, 0, 0);
      delay(500);
    }
  }
  infoPrintln("ADS initialization complete");

  infoPrintln("Setup complete!");
  infoPrintln("Entering main loop in 3 seconds...");
  delay(3000);  // Pause before entering loop
}

// Calculate the average of the values in the buffer
float calculateAverage() {
  float sum = 0;
  int count = bufferFilled ? WINDOW_SIZE : valueIndex;
  
  if (count == 0) return 0;
  
  for (int i = 0; i < count; i++) {
    sum += values[i];
  }
  return sum / count;
}

// Calculate the standard deviation of the values in the buffer
float calculateStdDev(float avg) {
  float sumSquares = 0;
  int count = bufferFilled ? WINDOW_SIZE : valueIndex;
  
  if (count <= 1) return 0;
  
  for (int i = 0; i < count; i++) {
    float diff = values[i] - avg;
    sumSquares += diff * diff;
  }
  return sqrt(sumSquares / (count - 1));
}

// Check if the signal is stable (low standard deviation)
bool isSignalStable(float stdDev) {
  return stdDev < STABILITY_THRESHOLD;
}

// Check if we should trigger based on rate limiting
bool canTrigger() {
  unsigned long currentTime = millis();
  
  // During testing with DISABLE_SLEEP, just check minimum interval
  if (DISABLE_SLEEP) {
    return (currentTime - lastTriggerTime >= MIN_TRIGGER_INTERVAL);
  }
  
  // Normal operation checks
  // Check if we've moved to a new hour
  if (currentTime - hourStartTime >= HOUR_MS) {
    hourStartTime = currentTime;
    triggerCount = 0;
  }
  
  // Check if we've exceeded the maximum triggers per hour
  if (triggerCount >= MAX_TRIGGERS_PER_HOUR) {
    return false;
  }
  
  // Check if we've waited the minimum interval
  if (currentTime - lastTriggerTime < MIN_TRIGGER_INTERVAL) {
    return false;
  }
  
  return true;
}

// Handle a detected change
void handleChange(float value, float avgValue, float changeAmount) {
  lastTriggerTime = millis();
  triggerCount++;
  
  // Update the baseline after a change
  baselineValue = avgValue;
  
  // Adjust the threshold based on the detected change, but keep it sensitive
  // For very small changes, keep the threshold at minimum
  if (changeAmount < MIN_THRESHOLD * 2) {
    currentThreshold = MIN_THRESHOLD;
  } else {
    currentThreshold = constrain(
      changeAmount * ADAPTIVE_FACTOR,
      MIN_THRESHOLD,
      MAX_THRESHOLD
    );
  }
  
  // Visual indication - Green for change detected
  setLEDColor(0, RGB_BRIGHTNESS, 0);
  
  // Initialize BLE for beacon advertising
  infoPrintln("Initializing BLE for beacon advertising...");
  initBLE();
  
  // Start beacon advertising
  advertiseBriefly();
  
  // Print change information in a compact format
  infoPrint("CHANGE: v=");
  infoPrint(value, 4);
  infoPrint(" avg=");
  infoPrint(avgValue, 4);
  infoPrint(" chg=");
  infoPrint(changeAmount, 4);
  infoPrint(" cnt=");
  infoPrint(triggerCount);
  infoPrint("/");
  infoPrintln(MAX_TRIGGERS_PER_HOUR);
  
  debugPrint(" thres=");
  debugPrintln(currentThreshold, 4);
  
  // Keep the LED on for 5 seconds
  delay(LED_ON_TIME);
  
  // Turn off LED
  setLEDColor(0, 0, 0);
}

// Heartbeat function to show the device is alive
void showHeartbeat() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastHeartbeatTime >= HEARTBEAT_INTERVAL) {
    // Flash the LED in soft orange/yellow (GRB order)
    // setLEDColor(RGB_BRIGHTNESS/2, RGB_BRIGHTNESS/2, 0);  // Soft orange/yellow
    // delay(200);
    setLEDColor(RGB_BRIGHTNESS/20, RGB_BRIGHTNESS/20, 0);  // Back to dim orange/yellow
    
    lastHeartbeatTime = currentTime;
    
    debugPrintln("Heartbeat");
  }
}

void loop()
{
  // Basic loop start info
  debugPrintln("\n=== LOOP START ===");
  debugPrint("Time: ");
  debugPrintln(millis());
  
  int16_t results;
  float multiplier = 0.1875F; /* ADS1115 @ +/- 6.144V gain (16-bit results) */
  
  // Read the ADS1115 with error handling
  try {
    results = ads.readADC_Differential_0_1();
  } catch (...) {
    infoPrintln("Error reading ADS1115!");
    setLEDColor(RGB_BRIGHTNESS, 0, 0); // Red to indicate error
    delay(1000);
    setLEDColor(0, 0, 0);
    delay(1000);
    return; // Skip this loop iteration
  }
  
  float currentValue = results * multiplier;  // Convert to mV
  
  // Add the current value to the circular buffer
  values[valueIndex] = currentValue;
  valueIndex = (valueIndex + 1) % WINDOW_SIZE;
  if (valueIndex == 0) {
    bufferFilled = true;
  }
  
  // Calculate statistics
  float avgValue = calculateAverage();
  float stdDev = calculateStdDev(avgValue);
  bool stable = isSignalStable(stdDev);
  
  // Establish baseline if not yet established or after a period of stability
  if (!baselineEstablished) {
    if (stable) {
      stableReadingsCount++;
      if (stableReadingsCount >= WINDOW_SIZE) {
        baselineValue = avgValue;
        baselineEstablished = true;
        infoPrint("Baseline set: ");
        infoPrintln(baselineValue, 4);
      }
    } else {
      stableReadingsCount = 0;
    }
  }
  
  // Calculate change relative to baseline or last average
  float changeAmount;
  if (baselineEstablished) {
    changeAmount = abs(avgValue - baselineValue);
  } else {
    changeAmount = abs(avgValue - lastAverage);
  }
  
  // Update the last average
  lastAverage = avgValue;
  
  // Single info line with important loop data
  infoPrint("ADS: raw=");
  infoPrint(results);
  infoPrint(" v=");
  infoPrint(currentValue, 4);
  infoPrint(" avg=");
  infoPrint(avgValue, 4);
  
  if (baselineEstablished) {
    infoPrint(" base=");
    infoPrint(baselineValue, 4);
    infoPrint(" chg=");
    infoPrint(changeAmount, 4);
    infoPrint(" thres=");
    infoPrint(currentThreshold, 4);
  }
  
  infoPrint(" stb=");
  infoPrintln(stable ? 1 : 0);
  
  // Check if we should trigger a change detection
  if (bufferFilled && baselineEstablished) {
    if (changeAmount > currentThreshold && canTrigger()) {
      infoPrintln("*** CHANGE DETECTED ***");
      handleChange(currentValue, avgValue, changeAmount);
      // Reset baseline after handling change
      baselineEstablished = false;
      stableReadingsCount = 0;
    } else if (stable && changeAmount > currentThreshold * 0.5 && canTrigger()) {
      infoPrintln("*** STABLE DRIFT DETECTED ***");
      handleChange(currentValue, avgValue, changeAmount);
      baselineEstablished = true;  // Keep baseline established but updated
    }
  }
    
  // Show heartbeat if needed
  showHeartbeat();
  
  // Save state to RTC memory
  saveStateToRTC();
  
  debugPrintln("=== LOOP END ===");
  
  // Turn off LED before sleep
  setLEDColor(0, 0, 0);
  
  #ifndef DISABLE_SLEEP
  // Deinitialize BLE before sleep
  deinitBLE();
  
  // Go to deep sleep
  esp_sleep_enable_timer_wakeup(SLEEP_DURATION * 1000000); // Convert to microseconds
  
  // Disable USB CDC during sleep to prevent issues on wake
  esp_sleep_pd_config(ESP_PD_DOMAIN_VDDSDIO, ESP_PD_OPTION_OFF);
  
  esp_deep_sleep_start();
  #else
  // Do NOT deinitialize BLE when sleep is disabled - keep the connection
  debugPrintln("Sleep disabled - pausing briefly");
  delay(500);  // Short delay between loop iterations
  #endif
}
