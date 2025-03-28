#define INCLUDE_ADS
//#define DEBUG 
//#define INFO
#define SLEEP_ENABLED  
//#define DEBUG_TRANSMISSION

#ifdef INCLUDE_ADS
#include <Adafruit_ADS1X15.h>
#endif

// Use USB CDC for serial on ESP32-S3
#define USE_USB_CDC

#include <Adafruit_NeoPixel.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>

#define I2C_SDA 34
#define I2C_SCL 33
#define RGB_LED 21  // Updated to match our working code
#define DELAYVAL 500
#define LED_ON_TIME 5000  // 5 seconds in milliseconds

// Power saving settings
#define SLEEP_DURATION 5 // Sleep duration in seconds
#define HEARTBEAT_INTERVAL 5000 // Heartbeat interval in milliseconds

// Change detection parameters
#define WINDOW_SIZE 3                 // Reduced window size to save memory
#define MIN_THRESHOLD 0.1875          // Minimum threshold set to exactly one bit of the ADS1115 (in mV)
#define MAX_THRESHOLD 50.0            // Lower maximum threshold (in mV)
#define ABSOLUTE_CHANGE_THRESHOLD 75.0 // Threshold for very large changes that should always trigger
#define ADAPTIVE_FACTOR 1.1           // Even gentler adaptation factor
#define STABILITY_THRESHOLD 0.2       // Slightly more permissive stability threshold (in mV)
#define ZERO_TO_NONZERO_THRESHOLD 5.0 // Special threshold for detecting changes from zero (in mV)
#define MIN_TRIGGER_INTERVAL 60000    // Minimum time between triggers (1 minute in ms)
#define HOUR_MS 3600000               // One hour in milliseconds
#define MAX_TRIGGERS_PER_HOUR 30      // Maximum number of triggers per hour

// MAC Address of receiver
uint8_t receiverMacAddress[] = {0xCC, 0x7B, 0x5C, 0xB9, 0xEF, 0x0C};

// Define a data structure
typedef struct message_struct {
  bool changeDetected;
} message_struct;

// Create a structured object
message_struct myData;

// Peer info
esp_now_peer_info_t peerInfo;

Adafruit_NeoPixel pixels(1, RGB_LED, NEO_GRB + NEO_KHZ800);

#ifdef INCLUDE_ADS
TwoWire s3i2c(0);
Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
// Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */
#endif

// Move large arrays to RTC memory to prevent stack overflow
RTC_DATA_ATTR float values[WINDOW_SIZE];            // Circular buffer for recent values
RTC_DATA_ATTR float values2[WINDOW_SIZE];           // Second circular buffer for recent values
RTC_DATA_ATTR int valueIndex = 0;                   // Current index in the circular buffer
RTC_DATA_ATTR float currentThreshold = MIN_THRESHOLD; // Current adaptive threshold
RTC_DATA_ATTR unsigned long lastTriggerTime = 0;    // Time of the last trigger
RTC_DATA_ATTR unsigned long hourStartTime = 0;      // Start time of the current hour
RTC_DATA_ATTR int triggerCount = 0;                 // Number of triggers in the current hour
RTC_DATA_ATTR float lastAverage = 0;                // Last calculated average
RTC_DATA_ATTR float lastAverage2 = 0;               // Last calculated average for second reading
RTC_DATA_ATTR bool bufferFilled = false;            // Flag to indicate if the buffer is filled
RTC_DATA_ATTR float baselineValue = 0;              // Baseline value for comparison
RTC_DATA_ATTR float baselineValue2 = 0;             // Baseline value for second reading
RTC_DATA_ATTR bool baselineEstablished = false;     // Flag to indicate if baseline is established
RTC_DATA_ATTR bool baselineEstablished2 = false;    // Flag to indicate if baseline is established for second reading
RTC_DATA_ATTR int stableReadingsCount = 0;          // Count of consecutive stable readings
RTC_DATA_ATTR int stableReadingsCount2 = 0;         // Count of consecutive stable readings for second reading
RTC_DATA_ATTR unsigned long lastHeartbeatTime = 0;  // Time of the last heartbeat
RTC_DATA_ATTR bool rtc_firstBoot = true;            // First boot flag

// WiFi status
bool wifiInitialized = false;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  #if defined(DEBUG) || defined(DEBUG_TRANSMISSION)
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  #endif
}

// Initialize WiFi and ESP-NOW
void initWiFi() {
  if (wifiInitialized) return;
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    #if defined(DEBUG) || defined(DEBUG_TRANSMISSION)
    Serial.println("Error initializing ESP-NOW");
    #endif
    return;
  }

  // Register the send callback
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    #if defined(DEBUG) || defined(DEBUG_TRANSMISSION)
    Serial.println("Failed to add peer");
    #endif
    return;
  }

  #if defined(DEBUG) || defined(DEBUG_TRANSMISSION)
  Serial.println("ESP-NOW initialized");
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
  #endif
  
  wifiInitialized = true;
}

// Deinitialize WiFi to save power
void deinitWiFi() {
  if (!wifiInitialized) return;
  
  esp_now_deinit();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  esp_wifi_stop();
  
  wifiInitialized = false;
  
  #if defined(DEBUG) || defined(DEBUG_TRANSMISSION)
  Serial.println("WiFi and ESP-NOW deinitialized");
  #endif
}

void setup()
{
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  
  #ifdef DEBUG
  Serial.println("Initializing LED...");
  #endif
  pixels.begin();           // Initialize NeoPixel
  pixels.setBrightness(20); // Slightly higher brightness for visibility
  pixels.clear();           // Turn off LED
  pixels.show();           
  delay(500);              // Give LED time to initialize
  
  // Double blink at startup
  #ifdef DEBUG
  Serial.println("Double blink startup sequence...");
  #endif
  // First blink
  pixels.setPixelColor(0, pixels.Color(0, 20, 0));  // GRB format: Red (G=0, R=20, B=0)
  pixels.show();
  delay(500);  // Longer on time
  pixels.clear();
  pixels.show();
  delay(500);  // Longer gap between blinks
  
  // Second blink
  pixels.setPixelColor(0, pixels.Color(0, 20, 0));  // GRB format: Red (G=0, R=20, B=0)
  pixels.show();
  delay(500);
  pixels.clear();
  pixels.show();
  delay(500);

  #ifdef DEBUG
  Serial.println("LED initialized!");
  #endif
  delay(1000);  // Give a moment before setting idle state

  // Set very dim red for normal operation
  pixels.setBrightness(10); // Reduce brightness for normal operation
  pixels.setPixelColor(0, pixels.Color(0, 1, 0));  // GRB format: Very dim red (G=0, R=1, B=0)
  pixels.show();

  // Initialize I2C first
  #ifdef INCLUDE_ADS
  #ifdef DEBUG
  Serial.println("Starting I2C initialization...");
  #endif
  s3i2c.begin(I2C_SDA, I2C_SCL, 100000);
  delay(500);  // Increased delay for I2C stabilization
  
  #ifdef DEBUG
  Serial.println("I2C initialized, attempting ADS begin...");
  #endif
  // Set the gain before begin
  ads.setGain(GAIN_TWOTHIRDS);  // +/- 6.144V range
  
  int retries = 0;
  while (!ads.begin(ADS1X15_ADDRESS, &s3i2c) && retries < 3) {
    #ifdef DEBUG
    Serial.println("ADS begin failed, retrying...");
    #endif
    delay(500);
    retries++;
  }
  
  if (retries >= 3) {
    #if defined(DEBUG) || defined(INFO)
    Serial.println("Failed to initialize ADS after 3 attempts!");
    #endif
  } else {
    #ifdef DEBUG
    Serial.println("ADS initialized successfully");
    #endif
  }
  #endif

  // Initialize the values array if first boot
  if (rtc_firstBoot) {
    for (int i = 0; i < WINDOW_SIZE; i++) {
      values[i] = 0;
      values2[i] = 0;
    }

    // Initialize timing variables
    hourStartTime = millis();
    lastTriggerTime = hourStartTime;
    lastHeartbeatTime = hourStartTime;
    rtc_firstBoot = false;
  }

  #ifdef DEBUG
  Serial.println("Setup complete");
  #endif
}

// Calculate the average of the values in the buffer
float calculateAverage(float* buffer, bool isFilled, int currentIndex) {
  float sum = 0;
  int count = isFilled ? WINDOW_SIZE : currentIndex;
  
  if (count == 0) return 0;
  
  for (int i = 0; i < count; i++) {
    sum += buffer[i];
  }
  return sum / count;
}

// Calculate the standard deviation of the values in the buffer
float calculateStdDev(float* buffer, float avg, bool isFilled, int currentIndex) {
  float sumSquares = 0;
  int count = isFilled ? WINDOW_SIZE : currentIndex;
  
  if (count <= 1) return 0;
  
  for (int i = 0; i < count; i++) {
    float diff = buffer[i] - avg;
    sumSquares += diff * diff;
  }
  return sqrt(sumSquares / (count - 1));
}

// Check if the signal is stable (low standard deviation)
bool isSignalStable(float stdDev) {
  return stdDev < STABILITY_THRESHOLD;
}

// New function to specifically detect a change from zero
bool isChangeFromZero(float avgValue, float baselineValue) {
  return baselineValue == 0 && abs(avgValue) > ZERO_TO_NONZERO_THRESHOLD;
}

// Check if we should trigger based on rate limiting
bool canTrigger() {
  unsigned long currentTime = millis();
  
  // Check if we've moved to a new hour
  if (currentTime - hourStartTime >= HOUR_MS) {
    hourStartTime = currentTime;
    triggerCount = 0;
    #ifdef DEBUG
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

// Send ESP-NOW message
void sendESPNowMessage() {
  // Initialize WiFi and ESP-NOW only when needed
  initWiFi();
  
  myData.changeDetected = true;
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(receiverMacAddress, (uint8_t *) &myData, sizeof(myData));
  
  #if defined(DEBUG) || defined(DEBUG_TRANSMISSION)
  if (result == ESP_OK) {
    Serial.println("ESP-NOW message sent successfully");
  } else {
    Serial.println("Error sending ESP-NOW message");
  }
  #endif
  
  // Wait a bit to ensure message is sent
  delay(100);
  
  // Turn off WiFi to save power
  deinitWiFi();
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
  
  // Visual indication - Blue for change detected (GRB format)
  pixels.setPixelColor(0, pixels.Color(0, 0, 50));  // Moderate blue brightness
  pixels.show();
  
  // Send ESP-NOW message
  sendESPNowMessage();
  
  #if defined(DEBUG) || defined(DEBUG_TRANSMISSION)
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
  pixels.clear();  // Use clear() instead of setting to black
  pixels.show();
}

// Heartbeat function to show the device is alive
void showHeartbeat() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastHeartbeatTime >= HEARTBEAT_INTERVAL) {
    // Flash the LED in slightly brighter red
    pixels.setPixelColor(0, pixels.Color(10, 0, 0));  // GRB format: Slightly brighter red
    pixels.show();
    delay(200);
    pixels.setPixelColor(0, pixels.Color(1, 0, 0));  // Back to very dim red
    pixels.show();
    
    lastHeartbeatTime = currentTime;
    
    #ifdef DEBUG
    Serial.println("Heartbeat");
    #endif
  }
}

void loop()
{
  #ifdef INCLUDE_ADS
  int16_t results;
  int16_t results2;
  float multiplier = 0.1875F; /* ADS1115 @ +/- 6.144V gain (16-bit results) */
  
  // Read both differential pairs
  results = ads.readADC_Differential_0_1();
  results2 = ads.readADC_Differential_2_3();
  float currentValue = results * multiplier;  // Convert to mV
  float currentValue2 = results2 * multiplier;  // Convert to mV
  
  // Add the current values to the circular buffers
  values[valueIndex] = currentValue;
  values2[valueIndex] = currentValue2;
  valueIndex = (valueIndex + 1) % WINDOW_SIZE;
  if (valueIndex == 0) {
    bufferFilled = true;
  }
  
  // Calculate statistics for both readings
  float avgValue = calculateAverage(values, bufferFilled, valueIndex);
  float avgValue2 = calculateAverage(values2, bufferFilled, valueIndex);
  float stdDev = calculateStdDev(values, avgValue, bufferFilled, valueIndex);
  float stdDev2 = calculateStdDev(values2, avgValue2, bufferFilled, valueIndex);
  
  // Check if signals are stable
  bool stable = isSignalStable(stdDev);
  bool stable2 = isSignalStable(stdDev2);
  
  // Establish baselines if not yet established or after a period of stability
  if (!baselineEstablished) {
    if (stable) {
      stableReadingsCount++;
      if (stableReadingsCount >= WINDOW_SIZE) {
        baselineValue = avgValue;
        baselineEstablished = true;
        #if defined(DEBUG) || defined(INFO)
        Serial.print("Baseline 1 established: ");
        Serial.print(baselineValue);
        Serial.println(" mV");
        #endif
      }
    } else {
      bool allZeros = true;
      for (int i = 0; i < (bufferFilled ? WINDOW_SIZE : valueIndex); i++) {
        if (values[i] != 0) {
          allZeros = false;
          break;
        }
      }
      
      if (allZeros && valueIndex >= 2) {
        baselineValue = 0;
        baselineEstablished = true;
        #if defined(DEBUG) || defined(INFO)
        Serial.println("Zero baseline 1 established from consecutive zero readings");
        #endif
      } else {
        stableReadingsCount = 0;
      }
    }
  }

  if (!baselineEstablished2) {
    if (stable2) {
      stableReadingsCount2++;
      if (stableReadingsCount2 >= WINDOW_SIZE) {
        baselineValue2 = avgValue2;
        baselineEstablished2 = true;
        #if defined(DEBUG) || defined(INFO)
        Serial.print("Baseline 2 established: ");
        Serial.print(baselineValue2);
        Serial.println(" mV");
        #endif
      }
    } else {
      bool allZeros = true;
      for (int i = 0; i < (bufferFilled ? WINDOW_SIZE : valueIndex); i++) {
        if (values2[i] != 0) {
          allZeros = false;
          break;
        }
      }
      
      if (allZeros && valueIndex >= 2) {
        baselineValue2 = 0;
        baselineEstablished2 = true;
        #if defined(DEBUG) || defined(INFO)
        Serial.println("Zero baseline 2 established from consecutive zero readings");
        #endif
      } else {
        stableReadingsCount2 = 0;
      }
    }
  }
  
  // Calculate changes relative to baselines or last averages
  float changeAmount;
  float changeAmount2;
  if (baselineEstablished) {
    changeAmount = abs(avgValue - baselineValue);
  } else {
    changeAmount = abs(avgValue - lastAverage);
  }
  
  if (baselineEstablished2) {
    changeAmount2 = abs(avgValue2 - baselineValue2);
  } else {
    changeAmount2 = abs(avgValue2 - lastAverage2);
  }
  
  // Update the last averages
  lastAverage = avgValue;
  lastAverage2 = avgValue2;
  
  #if defined(DEBUG) || defined(INFO)
  // Print current readings
  Serial.print("Differential 1: ");
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
  
  Serial.print("Differential 2: ");
  Serial.print(results2);
  Serial.print("(");
  Serial.print(currentValue2);
  Serial.print(" mV), Avg: ");
  Serial.print(avgValue2);
  Serial.print(" mV, StdDev: ");
  Serial.print(stdDev2);
  Serial.print(" mV, Threshold: ");
  Serial.print(currentThreshold);
  if (baselineEstablished2) {
    Serial.print(" mV, Baseline: ");
    Serial.print(baselineValue2);
    Serial.print(" mV, Change: ");
    Serial.print(changeAmount2);
  }
  Serial.println();
  #endif
  
  // Check if either reading should trigger a change detection
  bool shouldTrigger = false;
  if (bufferFilled && baselineEstablished && baselineEstablished2) {
    shouldTrigger = 
      (changeAmount > currentThreshold && canTrigger()) || 
      (changeAmount > ABSOLUTE_CHANGE_THRESHOLD && canTrigger()) ||
      (isChangeFromZero(avgValue, baselineValue) && canTrigger()) ||
      (baselineValue == 0 && !stable && abs(avgValue) > ZERO_TO_NONZERO_THRESHOLD * 3 && canTrigger()) ||
      (changeAmount2 > currentThreshold && canTrigger()) ||
      (changeAmount2 > ABSOLUTE_CHANGE_THRESHOLD && canTrigger()) ||
      (isChangeFromZero(avgValue2, baselineValue2) && canTrigger()) ||
      (baselineValue2 == 0 && !stable2 && abs(avgValue2) > ZERO_TO_NONZERO_THRESHOLD * 3 && canTrigger());
  }
  
  if (shouldTrigger) {
    #if defined(DEBUG) || defined(INFO)
    if (changeAmount > ABSOLUTE_CHANGE_THRESHOLD || changeAmount2 > ABSOLUTE_CHANGE_THRESHOLD) {
      Serial.println("Large absolute change detected!");
    } else if (isChangeFromZero(avgValue, baselineValue) || isChangeFromZero(avgValue2, baselineValue2)) {
      Serial.println("Change from zero baseline detected!");
    } else if ((baselineValue == 0 && !stable && abs(avgValue) > ZERO_TO_NONZERO_THRESHOLD * 3) ||
               (baselineValue2 == 0 && !stable2 && abs(avgValue2) > ZERO_TO_NONZERO_THRESHOLD * 3)) {
      Serial.println("Unstable transition from zero detected!");
    }
    #endif
    
    // Use the larger change amount for threshold adjustment
    float maxChange = max(changeAmount, changeAmount2);
    handleChange(max(avgValue, avgValue2), max(avgValue, avgValue2), maxChange);
    
    // Reset both baselines after handling change
    baselineEstablished = false;
    baselineEstablished2 = false;
    stableReadingsCount = 0;
    stableReadingsCount2 = 0;
  } else {
    // If either signal is stable but different from baseline, it might be a slow drift
    if ((stable && baselineEstablished && changeAmount > currentThreshold * 0.5 && canTrigger()) ||
        (stable2 && baselineEstablished2 && changeAmount2 > currentThreshold * 0.5 && canTrigger())) {
      #if defined(DEBUG) || defined(INFO)
      Serial.println("Stable drift detected - updating baseline");
      #endif
      float maxChange = max(changeAmount, changeAmount2);
      handleChange(max(avgValue, avgValue2), max(avgValue, avgValue2), maxChange);
      baselineEstablished = true;  // Keep baselines established but updated
      baselineEstablished2 = true;
    }
    
    // Show heartbeat if needed
    showHeartbeat();
  }
  #endif
  
  #ifdef SLEEP_ENABLED
  #ifdef DEBUG
  Serial.println("\n=== Preparing for sleep ===");
  Serial.println("1. Turning off LED...");
  #endif
  
  // Before sleep, update LED turn off code
  pixels.clear();
  pixels.show();
  
  #ifdef DEBUG
  Serial.println("2. Shutting down WiFi...");
  #endif
  
  // Properly shutdown WiFi and BT before sleep
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  esp_wifi_stop();
  btStop();
  
  #ifdef DEBUG
  Serial.println("3. Setting up wake timer...");
  #endif
  
  // Configure sleep wake-up source
  esp_sleep_enable_timer_wakeup(SLEEP_DURATION * 1000000); // Convert to microseconds
  
  #ifdef DEBUG
  Serial.println("4. Final preparations...");
  Serial.println("Going to sleep for " + String(SLEEP_DURATION) + " seconds");
  #endif

  Serial.flush();
  delay(100); // Give serial time to flush

  // Disconnect from USB CDC if enabled
  #ifdef USE_USB_CDC
  #ifdef DEBUG
  Serial.println("5. Closing Serial connection...");
  Serial.flush();
  delay(100); // Give serial time to flush
  #endif
  Serial.end();
  delay(100);
  #endif
  
  #ifdef DEBUG
  Serial.println("6. Entering deep sleep...");
  Serial.flush();
  delay(100); // Final delay to ensure all debug messages are sent
  #endif
  
  // Go to deep sleep
  esp_deep_sleep_start();
  #else
  // When sleep is disabled, add a small delay between measurements
  delay(SLEEP_DURATION * 1000); // Convert to milliseconds
  #endif
}
