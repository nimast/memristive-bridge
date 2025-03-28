/**
 *  @filename   :   epd7in5b_V2.ino
 *  @brief      :   7.5inch b V2 e-paper display demo
 *  @author     :   Yehui from Waveshare, adapted by user
 *
 *  Copyright (C) Waveshare      Nov 30 2020
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documnetation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to  whom the Software is
 * furished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <SPI.h>
#include "epd7in5b_V2.h"
#include "imagedata.h"
#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>

// Display resolution constants
#define SCREEN_WIDTH     EPD_WIDTH   // 800 pixels
#define SCREEN_HEIGHT    EPD_HEIGHT  // 480 pixels

// LED configuration
#define RGB_LED 21  // Onboard RGB LED pin
#define HEARTBEAT_INTERVAL 5000  // 5 seconds
unsigned long lastHeartbeatTime = 0;
bool initialDisplay = true;  // Flag to track if it's the first display update

// Timer constants and variables
#define AUTO_SWITCH_TIMEOUT 180000    // 180 seconds (3 minutes) of inactivity before auto-switching
#define CIRCUIT_DISPLAY_TIMEOUT 15000 // 15 seconds to display circuit before switching back to bridge
unsigned long lastActivityTime = 0;   // Time of last activity (signal received or display change)
unsigned long circuitStartTime = 0;   // Time when circuit mode was started

// Display mode - alternating between two modes
enum DisplayMode {
  BRIDGE_MODE,
  CIRCUIT_MODE
};

DisplayMode currentMode = BRIDGE_MODE;  // Start with bridge display
bool displayUpdateRequested = false;

// Initialize NeoPixel
Adafruit_NeoPixel pixels(1, RGB_LED, NEO_GRB + NEO_KHZ800);

// Constants for bridge geometry - adjusted for 7.5 inch display
const int BRIDGE_GAP = 300; // Reduced from 500 for smaller display
const int START_LEFT = 80;  // Adjusted from 150
const int START_RIGHT = 400; // Adjusted from 650
const int START_Y = 240;    // Adjusted from 400
const int NUM_ROOTS = 8;    // Reduced from 12 for smaller display
const int MAX_POINTS = 50;  // Reduced from 100
const int MAX_STEPS = 150;  // Reduced from 200
const int DISPLAY_DELAY = 20000; // 20 seconds

// Define the screen boundaries
const int MARGIN_TOP = 50;     // Adjusted top safety margin
const int MARGIN_BOTTOM = 10;  // Adjusted bottom safety margin

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

// Forward declarations
void drawBridgeDisplay(Epd &epd);
void drawChaoticCircuit(Epd &epd);
void heartbeatLED();
void flashLEDOnTransmission();
void drawTree(int x, int y, int len, float angle, int depth, Epd &epd);
float randomFloat(float min, float max);
int mirrorY(int y); // Function to mirror Y coordinate
void drawLine(int x0, int y0, int x1, int y1, Epd &epd, unsigned char color);
void drawRect(int x0, int y0, int x1, int y1, bool filled, Epd &epd, unsigned char color);
void drawCircle(int xc, int yc, int r, bool filled, Epd &epd, unsigned char color);
void drawText(int x, int y, const char* text, Epd &epd, unsigned char color);

// Callback function for ESP-NOW data reception
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  Serial.println("ESP-NOW callback called");
  message_struct receivedData;
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  
  Serial.println("ESP-NOW message received");
  
  if (receivedData.changeDetected) {
    Serial.println("Change detected! Updating display...");
    
    // Toggle display mode for each message received
    currentMode = (currentMode == BRIDGE_MODE) ? CIRCUIT_MODE : BRIDGE_MODE;
    Serial.print("Switching to mode: ");
    Serial.println(currentMode == BRIDGE_MODE ? "Bridge Display" : "Circuit Display");
    
    // Record start time if switching to circuit mode
    if (currentMode == CIRCUIT_MODE) {
      circuitStartTime = millis();
    }
    
    displayUpdateRequested = true;
    initialDisplay = true;  // Clear screen for next display
    
    // Update activity timestamp
    lastActivityTime = millis();
    
    // Flash LED on transmission received
    flashLEDOnTransmission();
  }
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("7.5inch e-Paper V2 demo");
  
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
  
  // Initialize e-Paper display
  Epd epd;
  if (epd.Init() != 0) {
    Serial.print("e-Paper init failed");
    return;
  }

  // Initialize activity timestamp
  lastActivityTime = millis();
  
  // Display the initial content
  if (currentMode == BRIDGE_MODE) {
    drawBridgeDisplay(epd);
  } else {
    drawChaoticCircuit(epd);
    circuitStartTime = millis(); // Start circuit display timer
  }
  
  randomSeed(analogRead(0));  // Initialize random seed
  
  Serial.println("Setup completed. Waiting for ESP-NOW messages...");
}

void loop() {
  // Get current time for various checks
  unsigned long currentTime = millis();
  
  // Check if we need to switch from circuit to bridge after 15 seconds
  if (currentMode == CIRCUIT_MODE && (currentTime - circuitStartTime >= CIRCUIT_DISPLAY_TIMEOUT)) {
    Serial.println("Circuit display timeout reached. Switching back to bridge...");
    currentMode = BRIDGE_MODE;
    displayUpdateRequested = true;
    initialDisplay = true;
    lastActivityTime = currentTime; // Update activity timestamp
  }
  
  // Check if it's been 3 minutes since the last activity
  if (currentTime - lastActivityTime >= AUTO_SWITCH_TIMEOUT) {
    Serial.println("Inactivity timeout reached. Switching display...");
    // Toggle display mode
    currentMode = (currentMode == BRIDGE_MODE) ? CIRCUIT_MODE : BRIDGE_MODE;
    
    // If switching to circuit mode, record the start time
    if (currentMode == CIRCUIT_MODE) {
      circuitStartTime = currentTime;
    }
    
    displayUpdateRequested = true;
    initialDisplay = true;
    lastActivityTime = currentTime; // Reset the activity timer
  }
  
  // Check if a display update has been requested
  if (displayUpdateRequested) {
    Serial.println("Updating display...");
    
    Epd epd;
    if (epd.Init() != 0) {
      Serial.print("e-Paper init failed");
      return;
    }
    
    // Draw based on current mode
    if (currentMode == BRIDGE_MODE) {
      Serial.println("Drawing indigenous bridge...");
      drawBridgeDisplay(epd);
    } else {
      Serial.println("Drawing chaotic circuit...");
      drawChaoticCircuit(epd);
      // Update circuit start time when actually drawing
      circuitStartTime = currentTime;
    }
    
    displayUpdateRequested = false;
  }
  
  // Check if it's time for a heartbeat
  if (currentTime - lastHeartbeatTime >= HEARTBEAT_INTERVAL) {
    heartbeatLED();
    lastHeartbeatTime = currentTime;
  }
  
  // Small delay to prevent CPU hogging
  delay(10);
}

// Function to mirror Y coordinate (flip vertically)
int mirrorY(int y) {
  return SCREEN_HEIGHT - y;
}

// Simple drawing function implementations for the 7.5 inch e-Paper
// These wrap the e-Paper API to simplify drawing operations
void drawLine(int x0, int y0, int x1, int y1, Epd &epd, unsigned char color) {
  // Create a small buffer for the line
  const int lineWidth = abs(x1-x0) + 1;
  const int lineHeight = abs(y1-y0) + 1;
  const int bufferSize = ((lineWidth > lineHeight ? lineWidth : lineHeight) + 7) / 8 * 8;
  unsigned char* lineBuffer = new unsigned char[bufferSize];
  memset(lineBuffer, 0, bufferSize);
  
  // Draw line into buffer (very simplified algorithm)
  int dx = abs(x1-x0), sx = x0<x1 ? 1 : -1;
  int dy = abs(y1-y0), sy = y0<y1 ? 1 : -1;
  int err = (dx>dy ? dx : -dy)/2, e2;
  
  for(;;) {
    // Set pixel in buffer
    int byteIdx = ((y0 - (y0 % 8))/8 * lineWidth) + x0;
    int bitIdx = y0 % 8;
    if (byteIdx < bufferSize) {
      lineBuffer[byteIdx] |= (1 << bitIdx);
    }
    
    if (x0==x1 && y0==y1) break;
    e2 = err;
    if (e2 >-dx) { err -= dy; x0 += sx; }
    if (e2 < dy) { err += dx; y0 += sy; }
  }
  
  // Display the buffer
  epd.Displaypart(lineBuffer, x0, mirrorY(y0), lineWidth, lineHeight, color);
  delete[] lineBuffer;
}

void drawRect(int x0, int y0, int x1, int y1, bool filled, Epd &epd, unsigned char color) {
  if (filled) {
    // Create a buffer for the rectangle
    const int width = abs(x1-x0);
    const int height = abs(y1-y0);
    const int bufferSize = ((width * height) + 7) / 8;
    unsigned char* rectBuffer = new unsigned char[bufferSize];
    
    // Fill the buffer
    memset(rectBuffer, 0xFF, bufferSize);
    
    // Display the buffer
    epd.Displaypart(rectBuffer, x0, mirrorY(y1), width, height, color);
    delete[] rectBuffer;
  } else {
    // Draw the four sides
    drawLine(x0, y0, x1, y0, epd, color);
    drawLine(x1, y0, x1, y1, epd, color);
    drawLine(x1, y1, x0, y1, epd, color);
    drawLine(x0, y1, x0, y0, epd, color);
  }
}

void drawCircle(int xc, int yc, int r, bool filled, Epd &epd, unsigned char color) {
  // Create a buffer for the circle
  const int size = 2*r+1;
  const int bufferSize = ((size * size) + 7) / 8;
  unsigned char* circleBuffer = new unsigned char[bufferSize];
  memset(circleBuffer, 0, bufferSize);
  
  // Draw circle into buffer
  for (int y = -r; y <= r; y++) {
    for (int x = -r; x <= r; x++) {
      if (x*x + y*y <= r*r) {
        if (filled || x*x + y*y >= (r-1)*(r-1)) {
          int bufX = x + r;
          int bufY = y + r;
          int byteIdx = ((bufY - (bufY % 8))/8 * size) + bufX;
          int bitIdx = bufY % 8;
          if (byteIdx < bufferSize) {
            circleBuffer[byteIdx] |= (1 << bitIdx);
          }
        }
      }
    }
  }
  
  // Display the buffer
  epd.Displaypart(circleBuffer, xc-r, mirrorY(yc+r), size, size, color);
  delete[] circleBuffer;
}

// Function to draw the bridge display
void drawBridgeDisplay(Epd &epd) {
  // Define tree positions and anchor points
  Point leftTrees[3];  // Reduced from 5 to 3 for smaller display
  Point rightTrees[3]; // Reduced from 5 to 3
  
  // Create tree positions
  for(int i = 0; i < 3; i++) {
    // Left bank trees - use mirrorY for vertical flipping
    leftTrees[i].x = 20 + i * 25;
    leftTrees[i].y = START_Y;
    
    // Right bank trees - use mirrorY for vertical flipping
    rightTrees[i].x = 420 + i * 25;
    rightTrees[i].y = START_Y;
  }
  
  // Draw banks with vertical mirroring
  // Left bank
  drawRect(0, START_Y, START_LEFT, START_Y + 240, true, epd, 0);
  // Right bank
  drawRect(START_RIGHT, START_Y, SCREEN_WIDTH, START_Y + 240, true, epd, 0);
  
  // Draw trees on banks - with vertical mirroring
  for(int i = 0; i < 3; i++) {
    drawTree(leftTrees[i].x, leftTrees[i].y, 50, -1.57, 4, epd);
    drawTree(rightTrees[i].x, rightTrees[i].y, 50, -1.57, 4, epd);
  }
  
  // Initialize roots with targets being trees on the opposite bank
  for(int i = 0; i < NUM_ROOTS; i++) {
    roots[i].numPoints = 1;
    roots[i].growing = true;
    roots[i].thickness = 2 + (i % 3); // Adjusted thickness
    
    // Choose which tree this root will target (distribute roots among trees)
    int targetTreeIdx = i % 3;
    // Choose source tree with better distribution
    int sourceTreeIdx = (i / 2) % 3;
    
    if(i % 2 == 0) {
      // Left side roots
      roots[i].points[0] = {leftTrees[sourceTreeIdx].x, START_Y};
      roots[i].dirX = 0.95;
      // Note: We're not flipping dirY here because the algorithm already handles it
      roots[i].dirY = ((i / 2) % 3 == 0) ? 0.12 : 0.22;
      roots[i].targetX = rightTrees[targetTreeIdx].x;
      
      if ((i / 2) % 3 < 2) {
        roots[i].targetY = START_Y - 20 - (i * 5);
      } else {
        roots[i].targetY = START_Y - 5 - (i * 3);
      }
      roots[i].isLeft = true;
    } else {
      // Right side roots
      roots[i].points[0] = {rightTrees[sourceTreeIdx].x, START_Y};
      roots[i].dirX = -0.95;
      roots[i].dirY = ((i / 2) % 3 == 1) ? 0.12 : 0.22;
      roots[i].targetX = leftTrees[targetTreeIdx].x;
      
      if ((i / 2) % 3 < 2) {
        roots[i].targetY = START_Y - 20 - (i * 5);
      } else {
        roots[i].targetY = START_Y - 5 - (i * 3);
      }
      roots[i].isLeft = false;
    }
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
      
      // Determine how far across the gap we are (as a percentage)
      float progressAcrossGap = 0.0;
      if (root->isLeft) {
        progressAcrossGap = (lastPoint->x - START_LEFT) / (float)(START_RIGHT - START_LEFT);
      } else {
        progressAcrossGap = (START_RIGHT - lastPoint->x) / (float)(START_RIGHT - START_LEFT);
      }
      
      // Update direction with influence toward target
      float targetInfluence = min(0.5, 0.3 + (1 - distToTarget / 300) * 0.3);
      float newDirX = (1 - targetInfluence) * root->dirX + targetInfluence * (dx / distToTarget);
      float newDirY = (1 - targetInfluence) * root->dirY + targetInfluence * (dy / distToTarget);
      
      // Apply downward bias in middle section
      if (progressAcrossGap > 0.3 && progressAcrossGap < 0.7) {
        newDirY += 0.2;
        if (newDirY < 0) newDirY = 0.1;
      }
      
      // Add random variation
      newDirX += randomFloat(-0.02, 0.02);
      float randY = randomFloat(-0.02, 0.02);
      newDirY += randY;
      
      // Normalize direction
      float len = sqrt(newDirX * newDirX + newDirY * newDirY);
      root->dirX = newDirX / len;
      root->dirY = newDirY / len;
      
      // Calculate new point
      Point newPoint = {
        (int16_t)(lastPoint->x + root->dirX * 10),
        (int16_t)(lastPoint->y + root->dirY * 10)
      };
      
      // Check boundaries
      if (newPoint.y < MARGIN_TOP || newPoint.y > SCREEN_HEIGHT - MARGIN_BOTTOM) {
        if (newPoint.y < MARGIN_TOP) {
          root->dirY = 0.2;
          newPoint.y = MARGIN_TOP + 5;
        } else if (newPoint.y > SCREEN_HEIGHT - MARGIN_BOTTOM) {
          root->growing = false;
          continue;
        }
      }
      
      // Draw root segment with vertical mirroring
      for(uint8_t t = 0; t < root->thickness; t++) {
        drawLine(lastPoint->x, lastPoint->y + t, newPoint.x, newPoint.y + t, epd, 0);
      }
      
      // Check if reached target
      float targetX = root->targetX;
      float targetY = root->targetY;
      float dx2 = newPoint.x - targetX;
      float dy2 = newPoint.y - targetY;
      float distToTree = sqrt(dx2*dx2 + dy2*dy2);
      
      if(distToTree < 15) {
        // Draw final connection
        for(uint8_t t = 0; t < root->thickness; t++) {
          drawLine(newPoint.x, newPoint.y + t, targetX, targetY + t, epd, 0);
        }
        connected[i] = true;
        root->growing = false;
        continue;
      }
      
      // Add point if still growing
      if(root->growing && root->numPoints < MAX_POINTS) {
        root->points[root->numPoints++] = newPoint;
        
        // Check growth bounds
        if((root->isLeft && newPoint.x > START_RIGHT) ||
           (!root->isLeft && newPoint.x < START_LEFT) ||
           newPoint.y < MARGIN_TOP || newPoint.y > START_Y + 200) {
          root->growing = false;
        }
      }
    }
  }
  
  // Add hanging roots - with vertical mirroring
  for(int i = 0; i < NUM_ROOTS; i++) {
    for(int j = 1; j < roots[i].numPoints; j += 2) {
      if(randomFloat(0, 1) < 0.5) {
        Point* point = &roots[i].points[j];
        int hangLen = random(10, 25);
        int hangX = random(-3, 3);
        
        for(int t = 0; t < 2; t++) {
          drawLine(point->x, point->y, point->x + hangX, point->y + hangLen + t, epd, 0);
        }
      }
    }
  }
  
  // Put display to sleep when done
  epd.Sleep();
}

// Function to draw a tree with vertical mirroring
void drawTree(int x, int y, int len, float angle, int depth, Epd &epd) {
  if (depth <= 0) return;
  
  int endX = x + len * cos(angle);
  int endY = y + len * sin(angle);
  drawLine(x, y, endX, endY, epd, 0);
  
  if (depth > 1) {
    int newLen = len * 0.7;
    drawTree(endX, endY, newLen, angle - 0.5, depth - 1, epd);
    drawTree(endX, endY, newLen, angle + 0.5, depth - 1, epd);
  }
}

// Function to draw text, adapted for the 7.5 inch display
void drawText(int x, int y, const char* text, Epd &epd, unsigned char color) {
  // Create a simple buffer for text - very simplified
  const int len = strlen(text);
  const int charWidth = 8;
  const int charHeight = 16;
  const int bufferWidth = len * charWidth;
  const int bufferSize = ((bufferWidth * charHeight) + 7) / 8;
  
  unsigned char* textBuffer = new unsigned char[bufferSize];
  memset(textBuffer, 0, bufferSize);
  
  // Display the buffer with text - this is simplified and would need proper font rendering
  epd.Displaypart(textBuffer, x, mirrorY(y+charHeight), bufferWidth, charHeight, color);
  delete[] textBuffer;
}

// Function to draw chaotic circuit diagrams with vertical mirroring
void drawChaoticCircuit(Epd &epd) {
  // Constants adjusted for 7.5 inch display
  const int MARGIN = 10;
  const int CANVAS_WIDTH = SCREEN_WIDTH;
  const int CANVAS_HEIGHT = SCREEN_HEIGHT;
  
  // Section dimensions
  const int TITLE_HEIGHT = 40;
  const int TITLE_LEFT_WIDTH = CANVAS_WIDTH / 2;
  const int COMPONENTS_WIDTH = 200;
  const int COMPONENTS_HEIGHT = 100;
  const int FOOTER_HEIGHT = 60;
  
  // Draw outer border rectangle
  drawRect(0, 0, CANVAS_WIDTH-1, CANVAS_HEIGHT-1, false, epd, 0);
  
  // Draw title section rectangles - side by side
  drawRect(0, 0, TITLE_LEFT_WIDTH-1, TITLE_HEIGHT-1, false, epd, 0);
  drawRect(TITLE_LEFT_WIDTH, 0, CANVAS_WIDTH-1, TITLE_HEIGHT-1, false, epd, 0);
  
  // Draw component list section rectangle - only on the left side
  drawRect(0, CANVAS_HEIGHT - COMPONENTS_HEIGHT - FOOTER_HEIGHT, 
          COMPONENTS_WIDTH-1, CANVAS_HEIGHT - FOOTER_HEIGHT-1, false, epd, 0);
  
  // Draw footer section rectangle - spans the entire width
  drawRect(0, CANVAS_HEIGHT - FOOTER_HEIGHT, CANVAS_WIDTH-1, CANVAS_HEIGHT-1, false, epd, 0);
  
  // Determine circuit complexity - scaled down for smaller display
  const int numNodes = random(4, 9);        // Reduced from 6-12
  const int numMemristors = random(2, 5);   // Reduced from 3-6
  const int numResistors = random(2, 5);    // Reduced from 3-7
  const int numCapacitors = random(1, 4);   // Reduced from 2-5
  const int numInductors = random(1, 3);    // Reduced from 1-3
  const int numDiodes = random(1, 3);       // Reduced from 1-4
  
  // Add text labels
  drawText(MARGIN * 2, TITLE_HEIGHT/2, "MEMRISTOR-BASED CIRCUIT", epd, 0);
  
  // Add a random circuit name
  const char* names[] = {"CHUA'S CIRCUIT", "MEMRISTIVE OSCILLATOR", "ROOT-BRIDGE NETWORK"};
  int nameIndex = random(0, 3);
  drawText(TITLE_LEFT_WIDTH + MARGIN, TITLE_HEIGHT/2, names[nameIndex], epd, 0);
  
  // Define circuit area boundaries
  struct {
    int x;
    int y;
    int width;
    int height;
  } circuitArea = {
    MARGIN,
    TITLE_HEIGHT + MARGIN,
    CANVAS_WIDTH - MARGIN*2,
    CANVAS_HEIGHT - TITLE_HEIGHT - FOOTER_HEIGHT - MARGIN*2
  };
  
  // Calculate bounds for circuit distribution
  struct {
    int left;
    int right;
    int top;
    int bottom;
  } circuitBounds = {
    circuitArea.x,
    circuitArea.x + circuitArea.width,
    circuitArea.y,
    CANVAS_HEIGHT - FOOTER_HEIGHT - MARGIN,
  };
  
  // Create nodes with positions
  Point nodes[16];  // Max 16 nodes for smaller display
  int actualNumNodes = min(numNodes, 16);
  int nodesCreated = 0;
  
  // Create perimeter nodes to ensure full space usage
  // Top nodes
  for(int i = 0; i < 2 && nodesCreated < actualNumNodes; i++) {
    int x = circuitBounds.left + (i+1) * (circuitArea.width/3);
    int y = circuitBounds.top + 30;
    nodes[nodesCreated].x = x;
    nodes[nodesCreated].y = y;
    nodesCreated++;
  }
  
  // Bottom nodes
  for(int i = 0; i < 2 && nodesCreated < actualNumNodes; i++) {
    int x = circuitBounds.left + (i+1) * (circuitArea.width/3);
    int y = circuitBounds.bottom - 30;
    nodes[nodesCreated].x = x;
    nodes[nodesCreated].y = y;
    nodesCreated++;
  }
  
  // Fill remaining nodes in center area
  if (nodesCreated < actualNumNodes) {
    int remaining = actualNumNodes - nodesCreated;
    int centerRows = ceil(sqrt(remaining));
    int centerCols = ceil(remaining / (float)centerRows);
    
    int centerWidth = circuitArea.width * 0.7;
    int centerHeight = (circuitBounds.bottom - circuitBounds.top) * 0.7;
    int centerLeft = circuitBounds.left + circuitArea.width * 0.15;
    int centerTop = circuitBounds.top + (circuitBounds.bottom - circuitBounds.top) * 0.15;
    
    int centerSpacingX = centerWidth / (centerCols + 1);
    int centerSpacingY = centerHeight / (centerRows + 1);
    
    int nodesPlaced = 0;
    for(int row = 0; row < centerRows && nodesPlaced < remaining; row++) {
      for(int col = 0; col < centerCols && nodesPlaced < remaining; col++) {
        int x = centerLeft + (col + 1) * centerSpacingX;
        int y = centerTop + (row + 1) * centerSpacingY;
        
        // Add some randomness to positions
        nodes[nodesCreated].x = x + random(-10, 10);
        nodes[nodesCreated].y = y + random(-10, 10);
        nodesCreated++;
        nodesPlaced++;
      }
    }
  }
  
  // Draw all nodes
  for (int i = 0; i < actualNumNodes; i++) {
    drawCircle(nodes[i].x, nodes[i].y, 3, true, epd, 0);
  }
  
  // Draw ground symbol at node 0
  Point groundNode = nodes[0];
  int groundX = groundNode.x;
  int groundY = groundNode.y + 20;
  drawLine(groundNode.x, groundNode.y, groundX, groundY, epd, 0);
  drawLine(groundX - 10, groundY, groundX + 10, groundY, epd, 0);
  drawLine(groundX - 6, groundY + 3, groundX + 6, groundY + 3, epd, 0);
  drawLine(groundX - 3, groundY + 6, groundX + 3, groundY + 6, epd, 0);
  
  // Helper function to get random node pair
  auto getRandomNodePair = [&]() -> std::pair<int, int> {
    int node1 = random(actualNumNodes);
    int node2;
    do {
      node2 = random(actualNumNodes);
    } while (node1 == node2);
    return {node1, node2};
  };
  
  // Draw memristors - simplified for 7.5 inch display
  for (int i = 0; i < numMemristors; i++) {
    auto [node1, node2] = getRandomNodePair();
    int compX = (nodes[node1].x + nodes[node2].x) / 2;
    int compY = (nodes[node1].y + nodes[node2].y) / 2;
    
    // Draw memristor as rectangle
    int width = 30;
    int height = 15;
    drawRect(compX - width/2, compY - height/2, compX + width/2, compY + height/2, false, epd, 0);
    drawText(compX - 5, compY - 6, "M", epd, 0);
    
    // Draw connecting lines
    drawLine(nodes[node1].x, nodes[node1].y, compX - width/2, compY, epd, 0);
    drawLine(nodes[node2].x, nodes[node2].y, compX + width/2, compY, epd, 0);
  }
  
  // Draw resistors - simplified zigzag
  for (int i = 0; i < numResistors; i++) {
    auto [node1, node2] = getRandomNodePair();
    int compX = (nodes[node1].x + nodes[node2].x) / 2;
    int compY = (nodes[node1].y + nodes[node2].y) / 2;
    
    // Draw zigzag resistor - simplified
    int startX = compX - 15;
    int endX = compX + 15;
    
    drawLine(nodes[node1].x, nodes[node1].y, startX, compY, epd, 0);
    drawLine(startX, compY, startX + 6, compY - 5, epd, 0);
    drawLine(startX + 6, compY - 5, startX + 12, compY + 5, epd, 0);
    drawLine(startX + 12, compY + 5, startX + 18, compY - 5, epd, 0);
    drawLine(startX + 18, compY - 5, startX + 24, compY + 5, epd, 0);
    drawLine(startX + 24, compY + 5, endX, compY, epd, 0);
    drawLine(endX, compY, nodes[node2].x, nodes[node2].y, epd, 0);
  }
  
  // Draw capacitors - simplified
  for (int i = 0; i < numCapacitors; i++) {
    auto [node1, node2] = getRandomNodePair();
    int compX = (nodes[node1].x + nodes[node2].x) / 2;
    int compY = (nodes[node1].y + nodes[node2].y) / 2;
    
    drawLine(nodes[node1].x, nodes[node1].y, compX - 5, compY, epd, 0);
    drawLine(compX - 5, compY - 8, compX - 5, compY + 8, epd, 0);
    drawLine(compX + 5, compY - 8, compX + 5, compY + 8, epd, 0);
    drawLine(compX + 5, compY, nodes[node2].x, nodes[node2].y, epd, 0);
  }
  
  // Add component counts in the component list section
  char countText[30];
  sprintf(countText, "MEMRISTORS: %d", numMemristors);
  drawText(20, CANVAS_HEIGHT - COMPONENTS_HEIGHT - FOOTER_HEIGHT + 20, countText, epd, 0);
  
  sprintf(countText, "RESISTORS: %d", numResistors);
  drawText(20, CANVAS_HEIGHT - COMPONENTS_HEIGHT - FOOTER_HEIGHT + 35, countText, epd, 0);
  
  sprintf(countText, "CAPACITORS: %d", numCapacitors);
  drawText(20, CANVAS_HEIGHT - COMPONENTS_HEIGHT - FOOTER_HEIGHT + 50, countText, epd, 0);
  
  // Add sample measurements in footer
  unsigned long currentTime = millis() / 1000;
  sprintf(countText, "TIME: %lu s", currentTime);
  drawText(20, CANVAS_HEIGHT - FOOTER_HEIGHT + 20, countText, epd, 0);
  
  int sampleNum = random(100, 999);
  sprintf(countText, "SAMPLE: %d", sampleNum);
  drawText(20, CANVAS_HEIGHT - FOOTER_HEIGHT + 35, countText, epd, 0);
  
  // Put display to sleep when done
  epd.Sleep();
}

// Function to generate random float between min and max
float randomFloat(float min, float max) {
  return min + (max - min) * random(1000) / 1000.0f;
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
