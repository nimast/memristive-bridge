#define INCLUDE_ADS TRUE
#define DEBUG TRUE  // Set to TRUE for debug output, FALSE for power saving

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

#define I2C_SDA 8
#define I2C_SCL 9
#define PIN 21
#define DELAYVAL 500
#define LED_ON_TIME 5000  // 5 seconds in milliseconds

// Power saving settings
#define CPU_FREQ_MHZ 80   // Lower CPU frequency (default is 240MHz)
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

Adafruit_NeoPixel pixels(1, PIN, NEO_RGB + NEO_KHZ800);

#ifdef INCLUDE_ADS
TwoWire s3i2c(0);
Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
// Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */
#endif

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

// WiFi status
bool wifiInitialized = false;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  #if DEBUG
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
    #if DEBUG
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
    #if DEBUG
    Serial.println("Failed to add peer");
    #endif
    return;
  }

  #if DEBUG
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
  
  #if DEBUG
  Serial.println("WiFi and ESP-NOW deinitialized");
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
  // Set CPU frequency to save power BEFORE initializing serial
  setCpuFrequencyMhz(CPU_FREQ_MHZ);
  
  #if DEBUG
  #ifdef USE_USB_CDC
  // For ESP32-S3 with USB CDC
  Serial.begin();  // No baud rate needed for USB CDC
  #else
  // Traditional UART serial
  Serial.begin(115200);
  #endif
  
  delay(2000);  // Longer delay for USB CDC initialization
  Serial.println("Hello! Starting in low power mode");
  Serial.print("CPU Frequency set to: ");
  Serial.print(getCpuFrequencyMhz());
  Serial.println(" MHz");
  #endif

  pixels.begin();
  pixels.setPixelColor(0, pixels.Color(5, 0, 0));  // Dim red for startup
  pixels.show();

  // Restore state if not first boot
  restoreStateFromRTC();

  #if DEBUG
  Serial.println("Getting differential reading from AIN0 (P) and AIN1 (N)");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");
  #endif

  // Initialize the values array if first boot
  if (rtc_firstBoot) {
    for (int i = 0; i < WINDOW_SIZE; i++) {
      values[i] = 0;
    }

    // Initialize timing variables
    hourStartTime = millis();
    lastTriggerTime = hourStartTime;
    lastHeartbeatTime = hourStartTime;
  }

  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
  #ifdef INCLUDE_ADS
  #if DEBUG
  Serial.println("Initializing ADS");
  #endif
  
  s3i2c.begin(I2C_SDA, I2C_SCL, 100000);

  if (!ads.begin(ADS1X15_ADDRESS, &s3i2c)) {
    #if DEBUG
    Serial.println("Failed to initialize ADS.");
    #endif
    while (1);
  }
  #endif
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

// Send ESP-NOW message
void sendESPNowMessage() {
  // Initialize WiFi and ESP-NOW only when needed
  initWiFi();
  
  myData.changeDetected = true;
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(receiverMacAddress, (uint8_t *) &myData, sizeof(myData));
  
  #if DEBUG
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
  
  // Visual indication - Green for change detected
  pixels.setPixelColor(0, pixels.Color(0, 50, 0));
  pixels.show();
  
  // Send ESP-NOW message
  sendESPNowMessage();
  
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
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  pixels.show();
}

// Heartbeat function to show the device is alive
void showHeartbeat() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastHeartbeatTime >= HEARTBEAT_INTERVAL) {
    // Flash the LED in stronger red
    pixels.setPixelColor(0, pixels.Color(50, 0, 0));
    pixels.show();
    delay(200);
    pixels.setPixelColor(0, pixels.Color(5, 0, 0));  // Back to dim red
    pixels.show();
    
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
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  pixels.show();
  
  // Go to deep sleep
  esp_sleep_enable_timer_wakeup(SLEEP_DURATION * 1000000); // Convert to microseconds
  
  #ifdef USE_USB_CDC
  // For ESP32-S3, disable USB CDC during sleep to prevent issues on wake
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  #endif
  
  esp_deep_sleep_start();
}
