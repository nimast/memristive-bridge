#define INCLUDE_ADS TRUE

#ifdef INCLUDE_ADS
#include <Adafruit_ADS1X15.h>
#endif

#include <Adafruit_NeoPixel.h>
#include <esp_now.h>
#include <WiFi.h>

#define I2C_SDA 34
#define I2C_SCL 33
#define PIN 21
#define DELAYVAL 500
#define LED_ON_TIME 5000  // 5 seconds in milliseconds

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

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup()
{
  delay(2000);
  Serial.begin(9600);
  delay(2000);
  Serial.println("Hello! gonna fire up LEDS");
  pixels.begin();

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
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
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("ESP-NOW initialized");
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());

  //s3i2c = TwoWire::
  Serial.println("started TwoWire");

  Serial.println("Getting differential reading from AIN0 (P) and AIN1 (N)");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");

  // Initialize the values array
  for (int i = 0; i < WINDOW_SIZE; i++) {
    values[i] = 0;
  }

  // Initialize timing variables
  hourStartTime = millis();
  lastTriggerTime = hourStartTime;

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
  Serial.println("inside include");
  s3i2c.begin(I2C_SDA, I2C_SCL, 100000);

  if (!ads.begin(ADS1X15_ADDRESS, &s3i2c)) {
    Serial.println("Failed to initialize ADS.");
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
    Serial.println("New hour started, resetting trigger count");
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
  myData.changeDetected = true;
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(receiverMacAddress, (uint8_t *) &myData, sizeof(myData));
  
  if (result == ESP_OK) {
    Serial.println("ESP-NOW message sent successfully");
  } else {
    Serial.println("Error sending ESP-NOW message");
  }
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
  
  // Keep the LED on for 5 seconds
  delay(LED_ON_TIME);
  
  // Turn off LED
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  pixels.show();
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
        Serial.print("Baseline established: ");
        Serial.print(baselineValue);
        Serial.println(" mV");
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
  
  // Check if we should trigger a change detection
  if (bufferFilled && baselineEstablished && changeAmount > currentThreshold && canTrigger()) {
    handleChange(currentValue, avgValue, changeAmount);
    // Reset baseline after handling change
    baselineEstablished = false;
    stableReadingsCount = 0;
  } else {
    // If signal is stable but different from baseline, it might be a slow drift
    if (stable && baselineEstablished && changeAmount > currentThreshold * 0.5 && canTrigger()) {
      Serial.println("Stable drift detected - updating baseline");
      handleChange(currentValue, avgValue, changeAmount);
      baselineEstablished = true;  // Keep baseline established but updated
    }
    
    // Normal LED behavior
    pixels.setPixelColor(0, pixels.Color(5, 0, 0));  // Dim red for normal operation
    pixels.show();
  }
  #endif
  
  delay(500);
}
