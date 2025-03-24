/* Includes ------------------------------------------------------------------*/
#include "DEV_Config.h"
#include "EPD.h"
#include "GUI_Paint.h"
#include "imagedata.h"
#include <stdlib.h>
#include "Display.h"
#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>

// LED configuration
#define RGB_LED 21  // Onboard RGB LED pin
#define HEARTBEAT_INTERVAL 5000  // 5 seconds
unsigned long lastHeartbeatTime = 0;
bool initialDisplay = true;  // Flag to track if it's the first display update

// Constants for bridge geometry
const int BRIDGE_GAP = 500;
const int START_LEFT = 150;
const int START_RIGHT = 650;
const int START_Y = 400;
const int NUM_ROOTS = 8;
const int MAX_POINTS = 100;
const int MAX_STEPS = 200;
const int DISPLAY_DELAY = 20000; // 20 seconds

// Structure definitions
struct Point {
    int16_t x;
    int16_t y;
};

struct Root {
    Point points[MAX_POINTS];
    uint8_t numPoints;
    bool growing;
    float dirX;
    float dirY;
    int16_t targetX;
    int16_t targetY;
    uint8_t thickness;
    bool isLeft;
};

// Define the ESP-NOW message structure to match the sender
typedef struct message_struct {
  bool changeDetected;
} message_struct;

Root roots[NUM_ROOTS];
bool displayUpdateRequested = false;

// Initialize NeoPixel
Adafruit_NeoPixel pixels(1, RGB_LED, NEO_GRB + NEO_KHZ800);

// Function declarations
void drawTree(int x, int y, int len, float angle, int depth);
float randomFloat(float min, float max);
void drawBridgeDisplay();
void heartbeatLED();
void flashLEDOnTransmission();

// Callback function for ESP-NOW data reception
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  Serial.println("ESP-NOW callback called");
  message_struct receivedData;
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  
  Serial.println("ESP-NOW message received");
  
  if (receivedData.changeDetected) {
    Serial.println("Change detected! Updating display...");
    displayUpdateRequested = true;
    
    // Flash LED on transmission received
    flashLEDOnTransmission();
  }
}

/* Entry point ----------------------------------------------------------------*/
void setup()
{
    Serial.begin(9600);
    
    // Initialize NeoPixel LED
    Serial.println("Initializing LED...");
    pixels.begin();           // Initialize NeoPixel
    pixels.setBrightness(50); // Medium brightness for visibility
    pixels.clear();           // Turn off LED
    pixels.show();
    delay(500);              // Give LED time to initialize
    
    // Double blink at startup to confirm LED is working
    // First blink - green
    pixels.setPixelColor(0, pixels.Color(0, 50, 0));  // GRB format: (Green, Red, Blue)
    pixels.show();
    delay(500);
    pixels.clear();
    pixels.show();
    delay(500);
    
    // Second blink - blue 
    pixels.setPixelColor(0, pixels.Color(0, 0, 50));  // GRB format: (Green, Red, Blue)
    pixels.show();
    delay(500);
    pixels.clear();
    pixels.show();
    delay(500);
    
    Serial.println("LED initialized!");
    
    // Set very dim red for normal operation
    pixels.setPixelColor(0, pixels.Color(0, 2, 0));  // Very dim red - GRB format
    pixels.show();
    
    DEV_Module_Init();
    
    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);
    
    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
    }
    
    // Register callback function for received data
    esp_now_register_recv_cb(OnDataRecv);
    
    Serial.println("ESP-NOW initialized");
    Serial.print("MAC Address: ");
    Serial.println(WiFi.macAddress());
    
    // Initialize display only once
    Display::begin();
    Display::background(WHITE);
    Display::show();
    
    Serial.println("Waiting for ESP-NOW messages...");
}

// Function to draw the bridge display
void drawBridgeDisplay() {
    // Only clear the background on first display
    if (initialDisplay) {
        Display::background(WHITE);
        initialDisplay = false;
    }
    
    // Draw banks - FIXED version
    // Left bank - specify end coordinates, not width/height
    Display::rect(0, START_Y, START_LEFT, START_Y + 600, true);    
    // Right bank - specify end coordinates, not width/height
    Display::rect(START_RIGHT, START_Y, EPD_13IN3K_WIDTH, START_Y + 600, true);
    
    // Draw trees on banks
    for(int i = 0; i < 5; i++) {
        drawTree(20 + i * 30, START_Y, 40, -1.57, 4);  // -1.57 ≈ -PI/2
    }
    for(int i = 0; i < 5; i++) {
        drawTree(670 + i * 30, START_Y, 40, -1.57, 4);
    }
    
    // Initialize roots
    for(int i = 0; i < NUM_ROOTS; i++) {
        roots[i].numPoints = 1;
        roots[i].growing = true;
        roots[i].thickness = 2 + (i % 3);
        
        if(i % 2 == 0) {
            // Left side roots
            roots[i].points[0] = {START_LEFT, START_Y};
            roots[i].dirX = 0.9;
            roots[i].dirY = -0.4;
            roots[i].targetX = START_RIGHT;
            roots[i].isLeft = true;
        } else {
            // Right side roots
            roots[i].points[0] = {START_RIGHT, START_Y};
            roots[i].dirX = -0.9;
            roots[i].dirY = -0.4;
            roots[i].targetX = START_LEFT;
            roots[i].isLeft = false;
        }
        roots[i].targetY = 300 + (i * 25);
    }
    
    // Grow the roots
    bool anyGrowing = true;
    bool connected[NUM_ROOTS] = {false};
    
    for(int step = 0; step < MAX_STEPS && anyGrowing; step++) {
        anyGrowing = false;
        
        for(int i = 0; i < NUM_ROOTS; i++) {
            if(!roots[i].growing || connected[i]) continue;
            
            anyGrowing = true;
            Root* root = &roots[i];
            Point* lastPoint = &root->points[root->numPoints - 1];
            
            // Calculate direction to target
            float dx = root->targetX - lastPoint->x;
            float dy = root->targetY - lastPoint->y;
            float distToTarget = sqrt(dx*dx + dy*dy);
            
            // Update direction
            float targetInfluence = 0.3;
            float newDirX = (1 - targetInfluence) * root->dirX + targetInfluence * (dx / distToTarget);
            float newDirY = (1 - targetInfluence) * root->dirY + targetInfluence * (dy / distToTarget);
            
            // Add upward bias and random variation
            newDirY -= 0.1;
            newDirX += randomFloat(-0.05, 0.05);
            newDirY += randomFloat(-0.05, 0.05);
            
            // Normalize direction
            float len = sqrt(newDirX * newDirX + newDirY * newDirY);
            root->dirX = newDirX / len;
            root->dirY = newDirY / len;
            
            // Calculate new point
            Point newPoint = {
                (int16_t)(lastPoint->x + root->dirX * 15),
                (int16_t)(lastPoint->y + root->dirY * 15)
            };
            
            // Draw root segment
            for(uint8_t t = 0; t < root->thickness; t++) {
                Display::line(lastPoint->x, lastPoint->y + t, 
                            newPoint.x, newPoint.y + t);
            }
            
            // Check for connections
            for(int j = 0; j < NUM_ROOTS; j++) {
                if(i == j || roots[i].isLeft == roots[j].isLeft) continue;
                
                for(int k = 0; k < roots[j].numPoints; k++) {
                    Point* point = &roots[j].points[k];
                    float connDx = newPoint.x - point->x;
                    float connDy = newPoint.y - point->y;
                    float dist = sqrt(connDx*connDx + connDy*connDy);
                    
                    if(dist < 40) {
                        uint8_t connThickness = max(root->thickness, roots[j].thickness);
                        for(uint8_t t = 0; t < connThickness; t++) {
                            Display::line(newPoint.x, newPoint.y + t,
                                       point->x, point->y + t);
                        }
                        connected[i] = true;
                        connected[j] = true;
                        root->growing = false;
                        break;
                    }
                }
                if(connected[i]) break;
            }
            
            // Add point if still growing
            if(root->growing && root->numPoints < MAX_POINTS) {
                root->points[root->numPoints++] = newPoint;
                
                // Check growth bounds
                if((root->isLeft && newPoint.x > START_RIGHT) ||
                   (!root->isLeft && newPoint.x < START_LEFT) ||
                   newPoint.y < 100 || newPoint.y > 500) {
                    root->growing = false;
                }
            }
        }
    }
    
    // Add hanging roots
    for(int i = 0; i < NUM_ROOTS; i++) {
        for(int j = 1; j < roots[i].numPoints; j += 4) {
            if(randomFloat(0, 1) < 0.3) {
                Point* point = &roots[i].points[j];
                int hangLen = random(10, 20);
                int hangX = random(-5, 5);
                for(int t = 0; t < 2; t++) {
                    Display::line(point->x, point->y,
                                point->x + hangX, point->y + hangLen + t);
                }
            }
        }
    }
    
    Display::show();
    
    // Reset the flag
    displayUpdateRequested = false;
}

// Function to create a subtle heartbeat effect on the LED
void heartbeatLED() {
    // First pulse
    pixels.setPixelColor(0, pixels.Color(0, 20, 0));  // GRB format: Red
    pixels.show();
    delay(50);
    pixels.setPixelColor(0, pixels.Color(0, 100, 0));  // Brighter red
    pixels.show();
    delay(100);
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));   // Off
    pixels.show();
    delay(100);
    
    // Second pulse (stronger)
    pixels.setPixelColor(0, pixels.Color(0, 30, 0));  // Medium red
    pixels.show();
    delay(50);
    pixels.setPixelColor(0, pixels.Color(0, 150, 0)); // Bright red
    pixels.show();
    delay(100);
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));   // Off
    pixels.show();
    
    // Set very dim red for normal operation
    pixels.setPixelColor(0, pixels.Color(0, 2, 0));  // Very dim red - GRB format
    pixels.show();
}

// Function to create a more elaborate flashing when receiving transmission
void flashLEDOnTransmission() {
    // Bright white flash
    for (int i = 0; i < 3; i++) {
        pixels.setPixelColor(0, pixels.Color(100, 200, 100)); // Bright white (GRB format)
        pixels.show();
        delay(50);
        pixels.setPixelColor(0, pixels.Color(0, 0, 0));       // Off
        pixels.show();
        delay(50);
    }
    
    // Blue to purple swirl
    for (int i = 0; i < 5; i++) {
        // Blue
        pixels.setPixelColor(0, pixels.Color(200, 0, 0));  // GRB format: Blue
        pixels.show();
        delay(100);
        // Purple
        pixels.setPixelColor(0, pixels.Color(100, 0, 200)); // GRB format: Purple
        pixels.show();
        delay(100);
    }
    
    // Fade out
    for (int i = 150; i > 0; i -= 10) {
        pixels.setPixelColor(0, pixels.Color(i/3, 0, i));
        pixels.show();
        delay(10);
    }
    
    // Set very dim red for normal operation
    pixels.setPixelColor(0, pixels.Color(0, 2, 0));  // Very dim red - GRB format
    pixels.show();
}

void drawTree(int x, int y, int len, float angle, int depth) {
    if (depth <= 0) return;
    
    int endX = x + len * cos(angle);
    int endY = y + len * sin(angle);
    Display::line(x, y, endX, endY);
    
    if (depth > 1) {
        int newLen = len * 0.7;
        drawTree(endX, endY, newLen, angle - 0.5, depth - 1);
        drawTree(endX, endY, newLen, angle + 0.5, depth - 1);
    }
}

float randomFloat(float min, float max) {
    return min + (max - min) * random(1000) / 1000.0f;
}

/* The main loop -------------------------------------------------------------*/
void loop()
{
  // Check if a display update has been requested
  if (displayUpdateRequested) {
    Serial.println("Updating display...");
    drawBridgeDisplay();
  }
  
  // Check if it's time for a heartbeat
  unsigned long currentTime = millis();
  if (currentTime - lastHeartbeatTime >= HEARTBEAT_INTERVAL) {
    heartbeatLED();
    lastHeartbeatTime = currentTime;
  }
  
  // Small delay to prevent CPU hogging
  delay(10);
}
