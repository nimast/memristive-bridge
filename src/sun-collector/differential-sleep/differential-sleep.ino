// Memristive bridge sender
// Copyright (c) Nimrod Astarhan 2025
// This sketch needs v3 of esp board lib to work with ESP32H2
// Last used 3.1.3

#define DEBUG TRUE  // Set to TRUE for debug output, FALSE for power saving
#define DISABLE_SLEEP TRUE  // Set to TRUE to disable sleep for debugging

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

// Power saving settings
#define CPU_FREQ_MHZ 160   // Changed to 160MHz which is supported
#define SLEEP_DURATION 5 // Sleep duration in seconds
#define HEARTBEAT_INTERVAL 5000 // Heartbeat interval in milliseconds

// Change detection parameters
#define WINDOW_SIZE 5                 // Smaller window size for faster response
#define MIN_THRESHOLD 0.1875          // Minimum threshold set to exactly one bit of the ADS1115 (in mV)
#define MAX_THRESHOLD 50.0            // Lower maximum threshold (in mV)
#define ADAPTIVE_FACTOR 1.1           // Even gentler adaptation factor
#define STABILITY_THRESHOLD 0.1       // Lower threshold to determine if signal is stable (in mV)
#define MIN_TRIGGER_INTERVAL 60000    // Minimum time between triggers (1 minute in ms)
#define HOUR_MS 3600000               // One hour in milliseconds
#define MAX_TRIGGERS_PER_HOUR 30      // Maximum number of triggers per hour

// BLE objects
BLEServer* pServer = nullptr;
BLECharacteristic* pCharacteristic = nullptr;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// Callback class for BLE server
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      #if DEBUG
      Serial.println("Device connected");
      #endif
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      #if DEBUG
      Serial.println("Device disconnected");
      #endif
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

// Initialize BLE
void initBLE() {
  // Create the BLE Device
  BLEDevice::init(DEVICE_NAME);

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create the BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  // Add a descriptor
  pCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  #if DEBUG
  Serial.println("BLE initialized");
  #endif
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

// Send BLE message
void sendBLEMessage() {
  if (!deviceConnected) {
    #if DEBUG
    Serial.println("No device connected, cannot send message");
    #endif
    return;
  }

  // Send the change detected message
  uint8_t message = 1;  // 1 indicates change detected
  pCharacteristic->setValue(&message, 1);
  pCharacteristic->notify();
  
  #if DEBUG
  Serial.println("BLE message sent successfully");
  #endif
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
  
  #if DEBUG
  Serial.println("State restored from RTC memory");
  #endif
}

void setup()
{
  // Initialize serial first, before anything else
  Serial.begin();
  delay(2000);  // Give serial time to initialize
  
  Serial.println("\n\nStarting setup...");
  
  // Set CPU frequency
  if (!setCpuFrequencyMhz(CPU_FREQ_MHZ)) {
    Serial.println("Failed to set CPU frequency!");
  } else {
    Serial.print("CPU Frequency set to: ");
    Serial.print(getCpuFrequencyMhz());
    Serial.println(" MHz");
  }

  // Test LED
  Serial.println("Testing LED...");
  setLEDColor(RGB_BRIGHTNESS, 0, 0);  // Red
  delay(1000);
  setLEDColor(0, RGB_BRIGHTNESS, 0);  // Green
  delay(1000);
  setLEDColor(0, 0, RGB_BRIGHTNESS);  // Blue
  delay(1000);
  setLEDColor(0, 0, 0);  // Off
  Serial.println("LED test complete");

  // Initialize BLE
  Serial.println("Initializing BLE...");
  initBLE();
  Serial.println("BLE initialization complete");

  // Restore state if not first boot
  Serial.println("Restoring state from RTC...");
  restoreStateFromRTC();
  Serial.println("State restoration complete");

  // Initialize I2C with explicit pins
  Serial.println("Initializing I2C...");
  s3i2c.end();  // End any existing I2C connection
  if (!s3i2c.begin(I2C_SDA, I2C_SCL, 100000)) {
    Serial.println("Failed to initialize I2C!");
    while (1) {
      setLEDColor(0, RGB_BRIGHTNESS, 0);  // Green to indicate error
      delay(500);
      setLEDColor(0, 0, 0);
      delay(500);
    }
  }
  Serial.println("I2C initialized successfully");

  // Initialize ADS
  Serial.println("Initializing ADS...");
  if (!ads.begin(ADS1X15_ADDRESS, &s3i2c)) {
    Serial.println("Failed to initialize ADS!");
    while (1) {
      setLEDColor(RGB_BRIGHTNESS, 0, 0);  // Red to indicate error
      delay(500);
      setLEDColor(0, 0, 0);
      delay(500);
    }
  }
  Serial.println("ADS initialization complete");

  Serial.println("Setup complete!");
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
  
  // Check if we've moved to a new hour
  if (currentTime - hourStartTime >= HOUR_MS) {
    hourStartTime = currentTime;
    triggerCount = 0;
    #if DEBUG
    Serial.println("New hour started, resetting trigger count");
    #endif
  }
  
  // Check if we've exceeded the maximum triggers per hour
  if (triggerCount >= MAX_TRIGGERS_PER_HOUR) {
    return false;
  }
  
  // Check if we've waited the minimum interval
  if (currentTime - lastTriggerTime < MIN_TRIGGER_INTERVAL) {
    return false;
  }
  
  // Calculate how much of the hour has passed (0.0 to 1.0)
  float hourProgress = (float)(currentTime - hourStartTime) / HOUR_MS;
  
  // Calculate the ideal number of triggers at this point in the hour
  int idealTriggers = round(hourProgress * MAX_TRIGGERS_PER_HOUR);
  
  // Allow triggering if we're behind the ideal pace
  return triggerCount < idealTriggers || (currentTime - lastTriggerTime >= MIN_TRIGGER_INTERVAL * 2);
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
  
  // Send BLE message
  sendBLEMessage();
  
  #if DEBUG
  Serial.print("CHANGE DETECTED! Value: ");
  Serial.print(value);
  Serial.print(" mV, Avg: ");
  Serial.print(avgValue);
  Serial.print(" mV, Change: ");
  Serial.print(changeAmount);
  Serial.print(" mV, New Threshold: ");
  Serial.print(currentThreshold);
  Serial.print(" mV, Trigger count: ");
  Serial.print(triggerCount);
  Serial.print("/");
  Serial.println(MAX_TRIGGERS_PER_HOUR);
  #endif
  
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
    setLEDColor(RGB_BRIGHTNESS/2, RGB_BRIGHTNESS/2, 0);  // Soft orange/yellow
    delay(200);
    setLEDColor(RGB_BRIGHTNESS/20, RGB_BRIGHTNESS/20, 0);  // Back to dim orange/yellow
    
    lastHeartbeatTime = currentTime;
    
    #if DEBUG
    Serial.println("Heartbeat");
    #endif
  }
}

void loop()
{
  #ifdef INCLUDE_ADS
  int16_t results;
  float multiplier = 0.1875F; /* ADS1115 @ +/- 6.144V gain (16-bit results) */
  
  results = ads.readADC_Differential_0_1();
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
  
  // Check if signal is stable
  bool stable = isSignalStable(stdDev);
  
  // Establish baseline if not yet established or after a period of stability
  if (!baselineEstablished) {
    if (stable) {
      stableReadingsCount++;
      if (stableReadingsCount >= WINDOW_SIZE) {
        baselineValue = avgValue;
        baselineEstablished = true;
        #if DEBUG
        Serial.print("Baseline established: ");
        Serial.print(baselineValue);
        Serial.println(" mV");
        #endif
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
  
  #if DEBUG
  // Print current readings
  Serial.print("Differential: ");
  Serial.print(results);
  Serial.print("(");
  Serial.print(currentValue);
  Serial.print(" mV), Avg: ");
  Serial.print(avgValue);
  Serial.print(" mV, StdDev: ");
  Serial.print(stdDev);
  Serial.print(" mV, Threshold: ");
  Serial.print(currentThreshold);
  if (baselineEstablished) {
    Serial.print(" mV, Baseline: ");
    Serial.print(baselineValue);
    Serial.print(" mV, Change: ");
    Serial.print(changeAmount);
  }
  Serial.println();
  #endif
  
  // Check if we should trigger a change detection
  if (bufferFilled && baselineEstablished && changeAmount > currentThreshold && canTrigger()) {
    handleChange(currentValue, avgValue, changeAmount);
    // Reset baseline after handling change
    baselineEstablished = false;
    stableReadingsCount = 0;
  } else {
    // If signal is stable but different from baseline, it might be a slow drift
    if (stable && baselineEstablished && changeAmount > currentThreshold * 0.5 && canTrigger()) {
      #if DEBUG
      Serial.println("Stable drift detected - updating baseline");
      #endif
      handleChange(currentValue, avgValue, changeAmount);
      baselineEstablished = true;  // Keep baseline established but updated
    }
    
    // Show heartbeat if needed
    showHeartbeat();
  }
  #endif
  
  // Save state to RTC memory
  saveStateToRTC();
  
  #if DEBUG
  Serial.println("Going to sleep for " + String(SLEEP_DURATION) + " seconds");
  Serial.flush(); // Make sure all serial data is sent before sleep
  #endif
  
  // Turn off LED before sleep
  setLEDColor(0, 0, 0);
  
  // Deinitialize BLE before sleep
  deinitBLE();
  
  #ifndef DISABLE_SLEEP
  // Go to deep sleep
  esp_sleep_enable_timer_wakeup(SLEEP_DURATION * 1000000); // Convert to microseconds
  
  #ifdef USE_USB_CDC
  // For ESP32-S3, disable USB CDC during sleep to prevent issues on wake
  esp_sleep_pd_config(ESP_PD_DOMAIN_VDDSDIO, ESP_PD_OPTION_OFF);
  #endif
  
  esp_deep_sleep_start();
  #else
  delay(SLEEP_DURATION * 1000);  // Just delay instead of sleeping
  #endif
}
