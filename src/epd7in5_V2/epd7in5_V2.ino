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
#define RGB_LED 2  // Changed from pin 21 to pin 2 which is more commonly available on ESP32
#define HEARTBEAT_INTERVAL 5000  // 5 seconds
unsigned long lastHeartbeatTime = 0;
bool initialDisplay = true;  // Flag to track if it's the first display update

// Timer constants and variables
#define AUTO_REFRESH_TIMEOUT 120000  // 180 seconds (3 minutes) timeout for auto refresh
#define CIRCUIT_DISPLAY_TIMEOUT 15000  // 15 seconds to show circuit before switching back
unsigned long lastUpdateTime = 0;  // Tracks time since last ESP-NOW update
unsigned long circuitDisplayStartTime = 0;  // Tracks when circuit display was started

// Display mode - alternating between two modes
enum DisplayMode {
  BRIDGE_MODE,
  CIRCUIT_MODE
};

DisplayMode currentMode = BRIDGE_MODE;  // Start with bridge display
bool displayUpdateRequested = false;

// Constants for bridge geometry
// Adjusted for 7.5inch display (800×480)
const int BRIDGE_GAP = 300;  // Reduced from 500 for smaller display
const int BRIDGE_GAP_WIDTH = 300; // Width between left and right banks
const int START_LEFT = 90;   // Reduced from 150
const int START_RIGHT = 360; // Reduced from 650
const int START_Y = 240;     // Reduced from 400
const int NUM_ROOTS = 8;     // Reduced from 12 for smaller display
const int MAX_POINTS = 100;
const int MAX_STEPS = 200;
const int DISPLAY_DELAY = 20000; // 20 seconds

// Define the screen boundaries
const int SCREEN_WIDTH = EPD_7IN5_V2_WIDTH;   // 800 pixels
const int SCREEN_HEIGHT = EPD_7IN5_V2_HEIGHT; // 480 pixels
const int MARGIN_TOP = 60;     // Reduced top safety margin
const int MARGIN_BOTTOM = 10;  // Reduced bottom safety margin

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

// Initialize NeoPixel
Adafruit_NeoPixel pixels(1, RGB_LED, NEO_GRB + NEO_KHZ800);

// Function declarations
void drawTree(int x, int y, int len, float angle, int depth, int barkWidth = 1);
float randomFloat(float min, float max);
void drawBridgeDisplay();
void drawChaoticCircuit();
void heartbeatLED();
void flashLEDOnTransmission();
void checkTimers();

// Circuit component types
enum ComponentType {
  MEMRISTOR,
  RESISTOR,
  CAPACITOR,
  INDUCTOR,
  DIODE,
  VOLTAGE_SOURCE,
  GROUND,
  NODE
};

// Circuit component structure
struct Component {
  ComponentType type;
  int x;
  int y;
  int node1X;
  int node1Y;
  int node2X;
  int node2Y;
  float angle;
};

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
    
    // If switching to circuit mode, record the start time
    if (currentMode == CIRCUIT_MODE) {
      circuitDisplayStartTime = millis();
    }
    
    displayUpdateRequested = true;
    initialDisplay = true;  // Clear screen for next display
    
    // Flash LED on transmission received
    flashLEDOnTransmission();
    
    // Update the last update time
    lastUpdateTime = millis();
  }
}

/* Entry point ----------------------------------------------------------------*/
void setup()
{
    delay(1000);
    Serial.begin(9600);
    delay(1000);
    
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
    // Enable vertical mirroring for 7.5" display
    Display::setMirrorVertical(true);
    Display::background(WHITE);
    
    // Draw initial display 
    Serial.println("Drawing initial display...");
    if (currentMode == BRIDGE_MODE) {
      drawBridgeDisplay();
    } else {
      drawChaoticCircuit();
      circuitDisplayStartTime = millis();
    }
    
    randomSeed(analogRead(0));  // Initialize random seed
    
    // Initialize timers
    lastUpdateTime = millis();
    
    Serial.println("Waiting for ESP-NOW messages...");
}

/* The main loop -------------------------------------------------------------*/
void loop()
{
  // Check if a display update has been requested
  if (displayUpdateRequested) {
    Serial.println("Updating display...");
    
    // Draw based on current mode
    if (currentMode == BRIDGE_MODE) {
      Serial.println("Drawing indigenous bridge...");
      drawBridgeDisplay();
    } else {
      Serial.println("Drawing chaotic circuit...");
      drawChaoticCircuit();
      // Record the start time of showing the circuit display
      circuitDisplayStartTime = millis();
    }
    
    displayUpdateRequested = false;
  }
  
  // Check timers for automatic display updates
  checkTimers();
  
  // Check if it's time for a heartbeat
  unsigned long currentTime = millis();
  if (currentTime - lastHeartbeatTime >= HEARTBEAT_INTERVAL) {
    heartbeatLED();
    lastHeartbeatTime = currentTime;
  }
  
  // Small delay to prevent CPU hogging
  delay(10);
}

// Function to check timers and update display as needed
void checkTimers() {
  unsigned long currentTime = millis();
  
  // Check if we need to auto-refresh the bridge display (no update for 180 seconds)
  if (currentTime - lastUpdateTime >= AUTO_REFRESH_TIMEOUT) {
    Serial.println("No updates for 3 minutes, refreshing bridge display...");
    
    // Reset to bridge mode regardless of current mode
    currentMode = BRIDGE_MODE;
    initialDisplay = true;
    
    // Update display
    drawBridgeDisplay();
    
    // Reset timer
    lastUpdateTime = currentTime;
  }
  
  // Check if we need to switch back from circuit to bridge (after 15 seconds)
  if (currentMode == CIRCUIT_MODE && 
      currentTime - circuitDisplayStartTime >= CIRCUIT_DISPLAY_TIMEOUT) {
    Serial.println("Circuit displayed for 15 seconds, switching back to bridge...");
    
    // Switch to bridge mode
    currentMode = BRIDGE_MODE;
    initialDisplay = true;
    
    // Update display
    drawBridgeDisplay();
  }
}

// Function to draw chaotic circuit diagrams
void drawChaoticCircuit() {
    if (initialDisplay) {
        Display::background(WHITE);
        initialDisplay = false;
    }
    
    // Constants
    const int MARGIN = 10;
    const int CANVAS_WIDTH = SCREEN_WIDTH;    // 800
    const int CANVAS_HEIGHT = SCREEN_HEIGHT;  // 480
    
    // Section dimensions - adjusted for smaller display
    const int TITLE_HEIGHT = 40;
    const int TITLE_LEFT_WIDTH = CANVAS_WIDTH / 2;
    const int COMPONENTS_WIDTH = 180;  // Width of the component list section
    const int COMPONENTS_HEIGHT = 100;
    const int FOOTER_HEIGHT = 60;      // Height of the footer section
    
    // Draw outer border rectangle 
    Display::rect(0, 0, CANVAS_WIDTH, CANVAS_HEIGHT, false);
    
    // Draw title section rectangles - side by side
    Display::rect(0, 0, TITLE_LEFT_WIDTH, TITLE_HEIGHT, false); // Left title box
    Display::rect(TITLE_LEFT_WIDTH, 0, CANVAS_WIDTH, TITLE_HEIGHT, false); // Right title box
    
    // Draw component list section rectangle - only on the left side
    Display::rect(0, CANVAS_HEIGHT - COMPONENTS_HEIGHT - FOOTER_HEIGHT, COMPONENTS_WIDTH, CANVAS_HEIGHT - FOOTER_HEIGHT, false);
    
    // Draw footer section rectangle - spans the entire width
    Display::rect(0, CANVAS_HEIGHT - FOOTER_HEIGHT, CANVAS_WIDTH, CANVAS_HEIGHT, false);
    
    // Determine circuit complexity
    const int numNodes = random(6, 11);            // 6-10 nodes (reduced)
    const int numMemristors = random(2, 5);        // 2-4 memristors (reduced)
    const int numResistors = random(2, 6);         // 2-5 resistors (reduced)
    const int numCapacitors = random(1, 4);        // 1-3 capacitors (reduced)
    const int numInductors = random(1, 3);         // 1-2 inductors (reduced)
    const int numDiodes = random(1, 3);            // 1-2 diodes (reduced)
    
    // Add title and random circuit name
    const char* names[] = {"CHUA'S CIRCUIT", "MEMRISTIVE OSCILLATOR", "ROOT-BRIDGE NETWORK", 
                         "CHAOTIC CIRCUIT", "NEUROMORPHIC EMULATOR", "ORGANIC COMPUTING NODE"};
    int nameIndex = random(0, 6);  // Choose a random name
    
    // Draw main title - left aligned in its box
    Paint_DrawString_EN(MARGIN * 2, TITLE_HEIGHT/2, "MEMRISTOR-BASED CIRCUIT", &Font16, WHITE, BLACK);
    
    // Draw circuit specific name - centered in right box
    int subtitleX = TITLE_LEFT_WIDTH + (TITLE_LEFT_WIDTH / 2) - 80;
    Paint_DrawString_EN(subtitleX, TITLE_HEIGHT/2, names[nameIndex], &Font12, WHITE, BLACK);
    
    // Define the circuit area boundaries
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
    
    // Calculate actual bounds for circuit distribution
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
    Point nodes[20];  // Max 20 nodes for safety
    int actualNumNodes = min(numNodes, 20);
    int nodesCreated = 0;
    
    // Helper function to add jitter to positions
    auto addJitter = [](int val, int jitterAmount) -> int {
        return val + random(-jitterAmount, jitterAmount);
    };
    
    // Create perimeter nodes to ensure full space usage
    // Top nodes
    for(int i = 0; i < 3 && nodesCreated < actualNumNodes; i++) {
        int x = circuitBounds.left + (i+1) * (circuitArea.width/4);
        int y = circuitBounds.top + 25;
        nodes[nodesCreated].x = x;
        nodes[nodesCreated].y = y;
        nodesCreated++;
    }
    
    // Bottom nodes
    for(int i = 0; i < 3 && nodesCreated < actualNumNodes; i++) {
        int x = circuitBounds.left + (i+1) * (circuitArea.width/4);
        int y = circuitBounds.bottom - 25;
        nodes[nodesCreated].x = x;
        nodes[nodesCreated].y = y;
        nodesCreated++;
    }
    
    // Left and right side nodes
    int availableHeight = circuitBounds.bottom - circuitBounds.top;
    for(int i = 0; i < 2 && nodesCreated < actualNumNodes; i++) {
        int y = circuitBounds.top + (i+1) * (availableHeight/3);
        
        // Left side
        nodes[nodesCreated].x = circuitBounds.left + 25;
        nodes[nodesCreated].y = y;
        nodesCreated++;
        
        if (nodesCreated < actualNumNodes) {
            // Right side
            nodes[nodesCreated].x = circuitBounds.right - 25;
            nodes[nodesCreated].y = y;
            nodesCreated++;
        }
    }
    
    // Fill remaining nodes in center area
    if (nodesCreated < actualNumNodes) {
        int remaining = actualNumNodes - nodesCreated;
        int centerRows = ceil(sqrt(remaining));
        int centerCols = ceil(remaining / (float)centerRows);
        
        int centerWidth = circuitArea.width * 0.7;
        int centerHeight = availableHeight * 0.7;
        int centerLeft = circuitBounds.left + circuitArea.width * 0.15;
        int centerTop = circuitBounds.top + availableHeight * 0.15;
        
        int centerSpacingX = centerWidth / (centerCols + 1);
        int centerSpacingY = centerHeight / (centerRows + 1);
        
        int nodesPlaced = 0;
        for(int row = 0; row < centerRows && nodesPlaced < remaining; row++) {
            for(int col = 0; col < centerCols && nodesPlaced < remaining; col++) {
                int x = centerLeft + (col + 1) * centerSpacingX;
                int y = centerTop + (row + 1) * centerSpacingY;
                
                // Add jitter to positions
                nodes[nodesCreated].x = addJitter(x, centerSpacingX * 0.25);
                nodes[nodesCreated].y = addJitter(y, centerSpacingY * 0.25);
                nodesCreated++;
                nodesPlaced++;
            }
        }
    }
    
    // Draw all nodes
    for (int i = 0; i < actualNumNodes; i++) {
        Display::circle(nodes[i].x, nodes[i].y, 3, true);
    }
    
    // Draw ground symbol at node 0
    Point groundNode = nodes[0];
    int groundX = groundNode.x;
    int groundY = groundNode.y + 20;
    Display::line(groundNode.x, groundNode.y, groundX, groundY);
    Display::line(groundX - 10, groundY, groundX + 10, groundY);
    Display::line(groundX - 7, groundY + 3, groundX + 7, groundY + 3);
    Display::line(groundX - 3, groundY + 6, groundX + 3, groundY + 6);
    
    // Add voltage source between node 1 and ground node
    int sourceX = (nodes[1].x + groundNode.x) / 2;
    int sourceY = (nodes[1].y + groundNode.y) / 2;
    int radius = 15;
    
    Display::circle(sourceX, sourceY, radius, false);
    Paint_DrawString_EN(sourceX - 3, sourceY - 5, "+", &Font12, WHITE, BLACK);
    Display::line(sourceX - 5, sourceY + 5, sourceX + 5, sourceY + 5);
    Display::line(nodes[1].x, nodes[1].y, sourceX - radius, sourceY);
    Display::line(groundNode.x, groundNode.y, sourceX + radius, sourceY);
    
    // Helper function to find a random node pair
    auto getRandomNodePair = [&]() -> std::pair<int, int> {
        int node1 = random(actualNumNodes);
        int node2;
        do {
            node2 = random(actualNumNodes);
        } while (node1 == node2);
        
        return {node1, node2};
    };
    
    // Draw memristors
    for (int i = 0; i < numMemristors; i++) {
        auto [node1, node2] = getRandomNodePair();
        int compX = (nodes[node1].x + nodes[node2].x) / 2;
        int compY = (nodes[node1].y + nodes[node2].y) / 2;
        
        // Draw memristor (rectangular with M)
        int width = 40;  // Reduced
        int height = 20; // Reduced
        
        Display::rect(compX - width/2, compY - height/2, compX + width/2, compY + height/2, false);
        Paint_DrawString_EN(compX - 3, compY - 5, "M", &Font12, WHITE, BLACK);
        
        // Draw connecting lines
        Display::line(nodes[node1].x, nodes[node1].y, compX - width/2, compY);
        Display::line(nodes[node2].x, nodes[node2].y, compX + width/2, compY);
    }
    
    // Draw resistors
    for (int i = 0; i < numResistors; i++) {
        auto [node1, node2] = getRandomNodePair();
        int compX = (nodes[node1].x + nodes[node2].x) / 2;
        int compY = (nodes[node1].y + nodes[node2].y) / 2;
        int width = 35;  // Reduced
        int height = 14; // Reduced
        
        // Draw zigzag resistor
        int startX = compX - width/2;
        int endX = compX + width/2;
        
        Display::line(nodes[node1].x, nodes[node1].y, startX, compY);
        Display::line(startX, compY, startX + 3, compY - height/2);
        Display::line(startX + 3, compY - height/2, startX + 11, compY + height/2);
        Display::line(startX + 11, compY + height/2, startX + 18, compY - height/2);
        Display::line(startX + 18, compY - height/2, startX + 25, compY + height/2);
        Display::line(startX + 25, compY + height/2, startX + 32, compY - height/2);
        Display::line(startX + 32, compY - height/2, endX, compY);
        Display::line(endX, compY, nodes[node2].x, nodes[node2].y);
    }
    
    // Draw capacitors
    for (int i = 0; i < numCapacitors; i++) {
        auto [node1, node2] = getRandomNodePair();
        int compX = (nodes[node1].x + nodes[node2].x) / 2;
        int compY = (nodes[node1].y + nodes[node2].y) / 2;
        int width = 30;  // Reduced
        int plateSpacing = 6;  // Reduced
        
        Display::line(nodes[node1].x, nodes[node1].y, compX - width/2, compY);
        Display::line(compX - width/2, compY - 10, compX - width/2, compY + 10);
        Display::line(compX - width/2 + plateSpacing, compY - 10, compX - width/2 + plateSpacing, compY + 10);
        Display::line(compX - width/2 + plateSpacing, compY, nodes[node2].x, nodes[node2].y);
    }
    
    // Draw inductors
    for (int i = 0; i < numInductors; i++) {
        auto [node1, node2] = getRandomNodePair();
        int compX = (nodes[node1].x + nodes[node2].x) / 2;
        int compY = (nodes[node1].y + nodes[node2].y) / 2;
        int width = 40;  // Reduced
        int loopRadius = 4;  // Reduced
        
        Display::line(nodes[node1].x, nodes[node1].y, compX - width/2, compY);
        
        // Draw loops
        for (int j = 0; j < 4; j++) {
            int loopX = compX - width/2 + 7 + j * 10;  // Reduced spacing
            Display::circle(loopX, compY, loopRadius, false);
        }
        
        Display::line(compX + width/2 - 3, compY, nodes[node2].x, nodes[node2].y);
    }
    
    // Draw diodes
    for (int i = 0; i < numDiodes; i++) {
        auto [node1, node2] = getRandomNodePair();
        int compX = (nodes[node1].x + nodes[node2].x) / 2;
        int compY = (nodes[node1].y + nodes[node2].y) / 2;
        int width = 20;  // Reduced
        int height = 14;  // Reduced
        
        Display::line(nodes[node1].x, nodes[node1].y, compX - width/2, compY);
        
        // Triangle
        Display::line(compX - width/2, compY - height/2, compX - width/2, compY + height/2);
        Display::line(compX - width/2, compY - height/2, compX + width/2, compY);
        Display::line(compX - width/2, compY + height/2, compX + width/2, compY);
        
        // Line
        Display::line(compX + width/2, compY - height/2, compX + width/2, compY + height/2);
        
        Display::line(compX + width/2, compY, nodes[node2].x, nodes[node2].y);
    }
    
    // Add component counts in the component list section
    const int componentStartX = 20;
    const int componentStartY = CANVAS_HEIGHT - COMPONENTS_HEIGHT - FOOTER_HEIGHT + 20;
    
    char countText[50];
    sprintf(countText, "MEMRISTORS: %d", numMemristors);
    Paint_DrawString_EN(componentStartX, componentStartY, countText, &Font12, WHITE, BLACK);
    
    sprintf(countText, "RESISTORS: %d", numResistors);
    Paint_DrawString_EN(componentStartX, componentStartY + 15, countText, &Font12, WHITE, BLACK);
    
    sprintf(countText, "CAPACITORS: %d", numCapacitors);
    Paint_DrawString_EN(componentStartX, componentStartY + 30, countText, &Font12, WHITE, BLACK);
    
    sprintf(countText, "INDUCTORS: %d", numInductors);
    Paint_DrawString_EN(componentStartX, componentStartY + 45, countText, &Font12, WHITE, BLACK);
    
    sprintf(countText, "DIODES: %d", numDiodes);
    Paint_DrawString_EN(componentStartX, componentStartY + 60, countText, &Font12, WHITE, BLACK);
    
    // Add sample measurements in footer
    const int footerY = CANVAS_HEIGHT - FOOTER_HEIGHT + 25;
    
    unsigned long currentTime = millis() / 1000;
    sprintf(countText, "TIME: %lu s", currentTime);
    Paint_DrawString_EN(20, footerY, countText, &Font12, WHITE, BLACK);
    
    int sampleNum = random(100, 1000);
    sprintf(countText, "SAMPLE: %d", sampleNum);
    Paint_DrawString_EN(20, footerY + 20, countText, &Font12, WHITE, BLACK);
    
    Display::show();
}

// Function to draw the bridge display
void drawBridgeDisplay() {
    // Only clear the background on first display
    if (initialDisplay) {
        Display::background(WHITE);
        initialDisplay = false;
    }
    
    // Randomize the gap position between original position and left side
    // Define the range for possible gap positions
    const int MIN_GAP_POSITION = 50;  // Minimum start position for left bank
    const int MAX_GAP_POSITION = 150; // Maximum start position for left bank
    
    // Define range for gap width
    const int MIN_GAP_WIDTH = 250;  // Minimum gap width
    const int MAX_GAP_WIDTH = 350;  // Maximum gap width
    
    // Randomly determine the gap position and width
    const int adjustedStartLeft = random(MIN_GAP_POSITION, MAX_GAP_POSITION);
    const int randomGapWidth = random(MIN_GAP_WIDTH, MAX_GAP_WIDTH); // Randomize the gap width
    const int adjustedStartRight = adjustedStartLeft + randomGapWidth; // Variable gap width
    
    // Random number of trees on each side (2-5)
    int numLeftTrees = random(2, 6);
    int numRightTrees = random(2, 6);
    
    // Define tree positions and anchor points
    Point leftTrees[5];  // Store left bank tree positions
    Point rightTrees[5]; // Store right bank tree positions
    
    // Create tree positions - with adjusted gap
    // Always have at least one tree closest to the bank on each side
    
    // Left bank trees - rightmost tree (closest to gap) is always present
    int leftTreeSpacing = adjustedStartLeft / (numLeftTrees + 1);
    for(int i = 0; i < numLeftTrees; i++) {
        leftTrees[i].x = 10 + i * leftTreeSpacing;
        leftTrees[i].y = START_Y;
    }
    
    // Right bank trees - leftmost tree (closest to gap) is always present
    int rightTreeSpacing = 30;
    for(int i = 0; i < numRightTrees; i++) {
        // Ensure the leftmost tree of right bank is not above the gap
        rightTrees[i].x = adjustedStartRight + 20 + i * rightTreeSpacing;
        rightTrees[i].y = START_Y;
    }
    
    // Draw banks - with adjusted positions
    // Left bank
    Display::rect(0, START_Y, adjustedStartLeft, START_Y + 240, true);    
    // Right bank
    Display::rect(adjustedStartRight, START_Y, SCREEN_WIDTH, START_Y + 240, true);
    
    // Draw trees on banks
    for(int i = 0; i < numLeftTrees; i++) {
        // Random bark width between 1-3
        int barkWidth = random(1, 4);
        drawTree(leftTrees[i].x, leftTrees[i].y, 60, -1.57, 4, barkWidth);
    }
    
    for(int i = 0; i < numRightTrees; i++) {
        // Random bark width between 1-3
        int barkWidth = random(1, 4);
        drawTree(rightTrees[i].x, rightTrees[i].y, 60, -1.57, 4, barkWidth);
    }
    
    // Initialize roots with targets being trees on the opposite bank
    for(int i = 0; i < NUM_ROOTS; i++) {
        roots[i].numPoints = 1;
        roots[i].growing = true;
        roots[i].thickness = 2 + (i % 3);  // Thinner roots for smaller display
        
        // Choose which tree this root will target (distribute roots among trees)
        int targetTreeIdx = i % numRightTrees;  // Updated to use variable tree count
        // Choose source tree with better distribution
        int sourceTreeIdx = (i / 2) % numLeftTrees;  // Updated to use variable tree count
        
        if(i % 2 == 0) {
            // Left side roots
            roots[i].points[0] = {leftTrees[sourceTreeIdx].x, START_Y};
            roots[i].dirX = 0.98;
            // Start with very minimal downward slope (almost horizontal)
            roots[i].dirY = ((i / 2) % 4 == 0) ? 0.15 : 0.2;
            roots[i].targetX = rightTrees[targetTreeIdx].x;
            
            if ((i / 2) % 4 < 2) {
                // Higher up the tree
                roots[i].targetY = START_Y - 20 - (i * 5);
            } else {
                // Near base of tree
                roots[i].targetY = START_Y - 5 - (i * 3);
            }
            roots[i].isLeft = true;
        } else {
            // Right side roots
            roots[i].points[0] = {rightTrees[sourceTreeIdx].x, START_Y};
            roots[i].dirX = -0.98;
            // Start with very minimal downward slope (almost horizontal)
            roots[i].dirY = ((i / 2) % 4 == 1) ? 0.15 : 0.2;
            roots[i].targetX = leftTrees[targetTreeIdx].x;
            
            if ((i / 2) % 4 < 2) {
                // Higher up the tree
                roots[i].targetY = START_Y - 20 - (i * 5);
            } else {
                // Near base of tree
                roots[i].targetY = START_Y - 5 - (i * 3);
            }
            roots[i].isLeft = false;
        }
    }
    
    // Update the progress across gap calculation to use adjusted positions
    // ... in the growing loop ...
    // Determine how far across the gap we are (as a percentage)
    // When calculating progressAcrossGap, use the adjusted positions:
    
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
            
            // Updated to use adjusted positions
            float progressAcrossGap = 0.0;
            if (root->isLeft) {
                // Left to right progress
                progressAcrossGap = (lastPoint->x - adjustedStartLeft) / (float)(adjustedStartRight - adjustedStartLeft);
            } else {
                // Right to left progress
                progressAcrossGap = (adjustedStartRight - lastPoint->x) / (float)(adjustedStartRight - adjustedStartLeft);
            }
            
            // Check if we're heading toward the middle or away from it
            bool headingTowardMiddle = false;
            if ((root->isLeft && progressAcrossGap < 0.5) || (!root->isLeft && progressAcrossGap < 0.5)) {
                headingTowardMiddle = true;
            }
            
            // Update direction - stronger influence as we get closer to tree
            float targetInfluence = min(0.5, 0.3 + (1 - distToTarget / 300) * 0.3);
            float newDirX = (1 - targetInfluence) * root->dirX + targetInfluence * (dx / distToTarget);
            float newDirY = (1 - targetInfluence) * root->dirY + targetInfluence * (dy / distToTarget);
            
            // Add varied bias based on root index for diversity
            if (distToTarget > 60 && distToTarget < 240) {
                // Apply a progressive downward bias based on how far across the gap we are
                float progressFactor = root->isLeft ? progressAcrossGap : (1 - progressAcrossGap);
                
                // More gradual bias - starts very small, increases more slowly at first,
                // then builds up more rapidly in the middle
                // Uses a modified sigmoid-like curve for more natural appearance
                float progressiveBias = 0.05 + pow(progressFactor, 1.5) * 0.9; // More gradual start
                
                // Apply increasingly more downward force as we progress
                float biasStrength = 0.1 + progressFactor * 0.5; // Increases with progress
                newDirY += progressiveBias * biasStrength;
                
                // Ensure roots always curve downward but with minimal initial slope
                if (progressFactor < 0.2) {
                    // Keep very minimal slope at the beginning
                    if (newDirY > 0.2) newDirY = 0.2;
                } else if (progressFactor < 0.5) {
                    // Gradually allow more downward bias
                    if (newDirY < 0.1) newDirY = 0.1;
                } else {
                    // Middle to end section - ensure sufficient downward curve
                    if (newDirY < 0.2) newDirY = 0.2;
                }
                
                // Different roots should have slightly different behaviors
                if (i % 4 < 2) {
                    newDirY += 0.05 * progressFactor; // Scales with progress
                } else {
                    newDirY += 0.1 * progressFactor;  // Scales with progress
                }
            }
            
            // When getting close to target tree, aim upward to climb it
            if (distToTarget < 60) { // Adjusted distance
                // Only pull upward if near the end of the journey and far enough across the gap
                if ((root->isLeft && progressAcrossGap > 0.75) || (!root->isLeft && progressAcrossGap < 0.25)) {
                    // Create a smooth transition to climbing 
                    float climbFactor = 1.0 - (distToTarget / 60.0); // 0 at 60 distance, 1 at 0 distance
                    
                    // Pull upward with increasing strength as we get closer
                    newDirY -= 0.2 * climbFactor;
                }
            }
            
            // Add random variation (reduced for straighter paths)
            newDirX += randomFloat(-0.02, 0.02);
            float randY = randomFloat(0, 0.03) * progressAcrossGap; // More variation in the middle
            newDirY += randY;
            
            // Normalize direction
            float len = sqrt(newDirX * newDirX + newDirY * newDirY);
            root->dirX = newDirX / len;
            root->dirY = newDirY / len;
            
            // Only enforce minimum slope in the middle sections
            float minSlope = 0.05 + progressAcrossGap * 0.2; // Gradual increase in minimum slope
            
            if (progressAcrossGap > 0.1 && progressAcrossGap < 0.9 && root->dirY < minSlope) {
                root->dirY = minSlope;
                // Re-normalize
                len = sqrt(root->dirX * root->dirX + root->dirY * root->dirY);
                root->dirX = root->dirX / len;
                root->dirY = root->dirY / len;
            }
            
            // Calculate new point - shorter steps for smaller display
            Point newPoint = {
                (int16_t)(lastPoint->x + root->dirX * 10),
                (int16_t)(lastPoint->y + root->dirY * 10)
            };
            
            // Check if the new point would be outside the screen boundaries
            if (newPoint.y < MARGIN_TOP || newPoint.y > SCREEN_HEIGHT - MARGIN_BOTTOM) {
                // If going outside vertically, redirect the root back into bounds
                if (newPoint.y < MARGIN_TOP) {
                    // Too high, force downward
                    root->dirY = 0.2;
                    newPoint.y = MARGIN_TOP + 5;
                } else if (newPoint.y > SCREEN_HEIGHT - MARGIN_BOTTOM) {
                    // Too low, stop growing
                    root->growing = false;
                    continue;
                }
            }
            
            // Draw root segment with appropriate thickness
            for(uint8_t t = 0; t < root->thickness; t++) {
                Display::line(lastPoint->x, lastPoint->y + t, 
                            newPoint.x, newPoint.y + t);
            }
            
            // Check if we've reached our target tree
            float targetX = root->targetX;
            float targetY = root->targetY;
            float dx2 = newPoint.x - targetX;
            float dy2 = newPoint.y - targetY;
            float distToTree = sqrt(dx2*dx2 + dy2*dy2);
            
            if(distToTree < 15) { // Reduced connection distance
                // Draw final connection to the tree
                for(uint8_t t = 0; t < root->thickness; t++) {
                    Display::line(newPoint.x, newPoint.y + t,
                               targetX, targetY + t);
                }
                connected[i] = true;
                root->growing = false;
                continue;
            }
            
            // Check for connections with other roots
            for(int j = 0; j < NUM_ROOTS; j++) {
                if(i == j || roots[i].isLeft == roots[j].isLeft) continue;
                
                for(int k = 0; k < roots[j].numPoints; k++) {
                    Point* point = &roots[j].points[k];
                    float connDx = newPoint.x - point->x;
                    float connDy = newPoint.y - point->y;
                    float dist = sqrt(connDx*connDx + connDy*connDy);
                    
                    if(dist < 40) { // Reduced connection distance
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
                
                // Updated to use adjusted positions
                if((root->isLeft && newPoint.x > adjustedStartRight) ||
                   (!root->isLeft && newPoint.x < adjustedStartLeft) ||
                   newPoint.y < 40 || newPoint.y > START_Y + 200) {
                    root->growing = false;
                }
            }
        }
    }
    
    // Add hanging roots - updated to use adjusted positions
    for(int i = 0; i < NUM_ROOTS; i++) {
        for(int j = 1; j < roots[i].numPoints; j += 2) {
            // Calculate progress across gap
            float progressAcrossGap = 0.0;
            if (roots[i].isLeft) {
                // Left to right progress
                progressAcrossGap = (roots[i].points[j].x - adjustedStartLeft) / (float)(adjustedStartRight - adjustedStartLeft);
            } else {
                // Right to left progress
                progressAcrossGap = (adjustedStartRight - roots[i].points[j].x) / (float)(adjustedStartRight - adjustedStartLeft);
            }
            
            // Higher probability and longer hangers in the middle
            float hangingProbability = (progressAcrossGap > 0.3 && progressAcrossGap < 0.7) ? 0.65 : 0.5;
            
            if(randomFloat(0, 1) < hangingProbability) {
                Point* point = &roots[i].points[j];
                
                // Base hanging length, longer in the middle
                int hangLen = random(10, 25); // Shorter for smaller display
                
                // Extra length in the middle
                if (progressAcrossGap > 0.3 && progressAcrossGap < 0.7) {
                    hangLen += random(10, 20); // Shorter for smaller display
                }
                
                int hangX = random(-3, 3);
                for(int t = 0; t < 2; t++) { // Thinner hangers
                    Display::line(point->x, point->y,
                                point->x + hangX, point->y + hangLen + t);
                }
            }
        }
    }
    
    // Add climbing tendrils on trees
    const int tendrilLength = 12; // Shorter tendrils
    
    // Left tree tendrils
    for(int i = 0; i < numLeftTrees; i++) {
        for(int j = 0; j < 3 + random(0, 3); j++) { // 3-5 tendrils per tree
            // Start from random points on the trunk
            int startY = leftTrees[i].y - random(5, 30);
            
            // Always end at the ground
            int endY = START_Y;
            
            // Create more elaborate climbing tendril with multiple segments
            int numSegments = 3 + random(0, 3); // 3-5 segments
            int lastX = leftTrees[i].x;
            int lastY = startY;
            
            // Create a meandering path to the ground
            for (int s = 0; s < numSegments; s++) {
                // Determine next point - more horizontal movement at the top, more vertical at the bottom
                float verticalBias = s / (float)numSegments; // Increases as we move down
                
                int nextX, nextY;
                if (s == numSegments - 1) {
                    // Last segment always reaches the ground
                    nextX = lastX + random(-8, 8);
                    nextY = endY;
                } else {
                    int horizontalRange = 15 * (1 - verticalBias);
                    int verticalStep = (endY - startY) / numSegments * (1 + 0.5 * verticalBias);
                    
                    nextX = lastX + random(-horizontalRange, horizontalRange);
                    nextY = lastY + verticalStep;
                }
                
                // Draw this segment with varying thickness
                int thickness = 1 + (j % 2); // 1-2 pixels thick
                for (int t = 0; t < thickness; t++) {
                    Display::line(lastX, lastY + t, nextX, nextY + t);
                }
                
                lastX = nextX;
                lastY = nextY;
            }
        }
    }
    
    // Right tree tendrils
    for(int i = 0; i < numRightTrees; i++) {
        for(int j = 0; j < 3 + random(0, 3); j++) { // 3-5 tendrils per tree
            // Start from random points on the trunk
            int startY = rightTrees[i].y - random(5, 30);
            
            // Always end at the ground
            int endY = START_Y;
            
            // Create more elaborate climbing tendril with multiple segments
            int numSegments = 3 + random(0, 3); // 3-5 segments
            int lastX = rightTrees[i].x;
            int lastY = startY;
            
            // Create a meandering path to the ground
            for (int s = 0; s < numSegments; s++) {
                // Determine next point - more horizontal movement at the top, more vertical at the bottom
                float verticalBias = s / (float)numSegments; // Increases as we move down
                
                int nextX, nextY;
                if (s == numSegments - 1) {
                    // Last segment always reaches the ground
                    nextX = lastX + random(-8, 8);
                    nextY = endY;
                } else {
                    int horizontalRange = 15 * (1 - verticalBias);
                    int verticalStep = (endY - startY) / numSegments * (1 + 0.5 * verticalBias);
                    
                    nextX = lastX + random(-horizontalRange, horizontalRange);
                    nextY = lastY + verticalStep;
                }
                
                // Draw this segment with varying thickness
                int thickness = 1 + (j % 2); // 1-2 pixels thick
                for (int t = 0; t < thickness; t++) {
                    Display::line(lastX, lastY + t, nextX, nextY + t);
                }
                
                lastX = nextX;
                lastY = nextY;
            }
        }
    }
    
    // Ensure all roots are connected - use adjusted positions
    for(int i = 0; i < NUM_ROOTS; i++) {
        if(!connected[i] && roots[i].numPoints > 0) {
            // Find the closest root or tree from the opposite side to connect to
            int closestIdx = -1;
            float minDist = 1000;
            Point* lastPoint = &roots[i].points[roots[i].numPoints - 1];
            
            // Try to connect to another root first
            for(int j = 0; j < NUM_ROOTS; j++) {
                if(roots[i].isLeft == roots[j].isLeft || roots[j].numPoints == 0) continue;
                
                for(int k = 0; k < roots[j].numPoints; k++) {
                    float dx = lastPoint->x - roots[j].points[k].x;
                    float dy = lastPoint->y - roots[j].points[k].y;
                    float dist = sqrt(dx*dx + dy*dy);
                    
                    if(dist < minDist) {
                        minDist = dist;
                        closestIdx = j;
                    }
                }
            }
            
            // If we found a close enough root, connect to it
            if(closestIdx >= 0 && minDist < 120) { // Reduced distance
                Point* connPoint = nullptr;
                float shortestDist = 1000;
                
                // Find the closest point in the other root
                for(int k = 0; k < roots[closestIdx].numPoints; k++) {
                    float dx = lastPoint->x - roots[closestIdx].points[k].x;
                    float dy = lastPoint->y - roots[closestIdx].points[k].y;
                    float dist = sqrt(dx*dx + dy*dy);
                    
                    if(dist < shortestDist) {
                        shortestDist = dist;
                        connPoint = &roots[closestIdx].points[k];
                    }
                }
                
                if(connPoint) {
                    // Draw connection with smooth curve
                    int midX = (lastPoint->x + connPoint->x) / 2;
                    int midY = (lastPoint->y + connPoint->y) / 2 + random(-10, 10);
                    
                    // Draw curved connection path
                    uint8_t connThickness = max(roots[i].thickness, roots[closestIdx].thickness);
                    for(uint8_t t = 0; t < connThickness; t++) {
                        Display::line(lastPoint->x, lastPoint->y + t, midX, midY + t);
                        Display::line(midX, midY + t, connPoint->x, connPoint->y + t);
                    }
                    
                    connected[i] = true;
                    connected[closestIdx] = true;
                }
            }
        }
    }
    
    Display::show();
    
    // Reset the flag
    displayUpdateRequested = false;
}

// Function to create a subtle heartbeat effect on the LED - enhanced brightness
void heartbeatLED() {
    // Debug info
    Serial.println("Heartbeat LED...");
    
    // Increased brightness values
    
    // First pulse
    pixels.setPixelColor(0, pixels.Color(0, 100, 0));  // GRB format: Red - increased brightness
    pixels.show();
    delay(50);
    pixels.setPixelColor(0, pixels.Color(0, 200, 0));  // Brighter red - increased brightness
    pixels.show();
    delay(100);
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));   // Off
    pixels.show();
    delay(100);
    
    // Second pulse (stronger)
    pixels.setPixelColor(0, pixels.Color(0, 150, 0));  // Medium red - increased brightness
    pixels.show();
    delay(50);
    pixels.setPixelColor(0, pixels.Color(0, 255, 0)); // Bright red - maximum brightness
    pixels.show();
    delay(100);
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));   // Off
    pixels.show();
    
    // Set very dim red for normal operation
    pixels.setPixelColor(0, pixels.Color(0, 10, 0));  // Very dim red - increased from 2 to 10
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

void drawTree(int x, int y, int len, float angle, int depth, int barkWidth) {
    if (depth <= 0) return;
    
    int endX = x + len * cos(angle);
    int endY = y + len * sin(angle);
    
    // Draw the line with the specified bark width
    for (int i = 0; i < barkWidth; i++) {
        int offsetX = i/2 * (i % 2 == 0 ? -1 : 1);
        Display::line(x + offsetX, y, endX + offsetX, endY);
    }
    
    if (depth > 1) {
        // Reduce bark width as we go up the tree
        int newBarkWidth = max(1, barkWidth - 1);
        
        // Calculate new length with some random variation
        int newLen = len * (0.65 + randomFloat(0, 0.1));
        
        // Create more diverse branching patterns
        float leftAngle = angle - (0.4 + randomFloat(0, 0.2));
        float rightAngle = angle + (0.4 + randomFloat(0, 0.2));
        
        // Sometimes add a middle branch for more diversity
        if (depth > 2 && random(100) < 30) {
            float middleAngle = angle + randomFloat(-0.1, 0.1);
            drawTree(endX, endY, newLen, middleAngle, depth - 1, newBarkWidth);
        }
        
        // Draw left and right branches
        drawTree(endX, endY, newLen, leftAngle, depth - 1, newBarkWidth);
        drawTree(endX, endY, newLen, rightAngle, depth - 1, newBarkWidth);
    }
}

float randomFloat(float min, float max) {
    return min + (max - min) * random(1000) / 1000.0f;
}
