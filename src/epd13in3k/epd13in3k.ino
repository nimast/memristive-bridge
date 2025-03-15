/* Includes ------------------------------------------------------------------*/
#include "DEV_Config.h"
#include "EPD.h"
#include "GUI_Paint.h"
#include "imagedata.h"
#include <stdlib.h>
#include "Display.h"
#include <esp_now.h>
#include <WiFi.h>

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

// Function declarations
void drawTree(int x, int y, int len, float angle, int depth);
float randomFloat(float min, float max);
void drawBridgeDisplay();

// Callback function for ESP-NOW data reception
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  Serial.println("ESP-NOW callback called");
  message_struct receivedData;
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  
  Serial.println("ESP-NOW message received");
  
  if (receivedData.changeDetected) {
    Serial.println("Change detected! Updating display...");
    displayUpdateRequested = true;
  }
}

/* Entry point ----------------------------------------------------------------*/
void setup()
{
    Serial.begin(9600);
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
    // Reset the display without reinitializing
    Display::background(WHITE);
    
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
    
    // Wait for the specified delay time
    delay(DISPLAY_DELAY);
    
    Display::background(WHITE);
    Display::show();

    // Reset the flag
    displayUpdateRequested = false;
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
  
  // Small delay to prevent CPU hogging
  delay(100);
}
