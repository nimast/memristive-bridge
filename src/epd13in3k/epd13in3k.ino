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

// Display mode - alternating between two modes
enum DisplayMode {
  BRIDGE_MODE,
  CIRCUIT_MODE
};

DisplayMode currentMode = BRIDGE_MODE;  // Start with bridge display
bool displayUpdateRequested = false;

// Constants for bridge geometry
const int BRIDGE_GAP = 500;
const int START_LEFT = 150;
const int START_RIGHT = 650;
const int START_Y = 400;
const int NUM_ROOTS = 12;  // Increased from 8 to 12 roots
const int MAX_POINTS = 100;
const int MAX_STEPS = 200;
const int DISPLAY_DELAY = 20000; // 20 seconds

// Define the screen boundaries
const int SCREEN_WIDTH = EPD_13IN3K_WIDTH;  // 1333 pixels
const int SCREEN_HEIGHT = EPD_13IN3K_HEIGHT;  // 1000 pixels
const int MARGIN_TOP = 100;  // Top safety margin
const int MARGIN_BOTTOM = 20;  // Bottom safety margin

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
void drawTree(int x, int y, int len, float angle, int depth);
float randomFloat(float min, float max);
void drawBridgeDisplay();
void drawChaoticCircuit();
void heartbeatLED();
void flashLEDOnTransmission();

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
    
    displayUpdateRequested = true;
    initialDisplay = true;  // Clear screen for next display
    
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
    
    // Draw initial display 
    Serial.println("Drawing initial display...");
    if (currentMode == BRIDGE_MODE) {
      drawBridgeDisplay();
    } else {
      drawChaoticCircuit();
    }
    
    randomSeed(analogRead(0));  // Initialize random seed
    
    Serial.println("Waiting for ESP-NOW messages...");
}

// Function to draw chaotic circuit diagrams
void drawChaoticCircuit() {
    if (initialDisplay) {
        Display::background(WHITE);
        initialDisplay = false;
    }
    
    // Constants
    const int MARGIN = 20;
    const int CANVAS_WIDTH = EPD_13IN3K_WIDTH;
    const int CANVAS_HEIGHT = EPD_13IN3K_HEIGHT;
    
    // Section dimensions
    const int TITLE_HEIGHT = 70;
    const int TITLE_LEFT_WIDTH = CANVAS_WIDTH / 2;
    const int COMPONENTS_WIDTH = 300;  // Width of the component list section
    const int COMPONENTS_HEIGHT = 150;
    const int FOOTER_HEIGHT = 100;  // Height of the footer section
    
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
    const int numNodes = random(6, 13);            // 6-12 nodes
    const int numMemristors = random(3, 7);        // 3-6 memristors
    const int numResistors = random(3, 8);         // 3-7 resistors
    const int numCapacitors = random(2, 6);        // 2-5 capacitors
    const int numInductors = random(1, 4);         // 1-3 inductors
    const int numDiodes = random(1, 5);            // 1-4 diodes
    
    // Add title and random circuit name
    const char* names[] = {"CHUA'S CIRCUIT", "MEMRISTIVE OSCILLATOR", "ROOT-BRIDGE NETWORK", 
                         "CHAOTIC CIRCUIT", "NEUROMORPHIC EMULATOR", "ORGANIC COMPUTING NODE"};
    int nameIndex = random(0, 6);  // Choose a random name
    
    // Draw main title - left aligned in its box
    Paint_DrawString_EN(MARGIN * 2, TITLE_HEIGHT/2, "MEMRISTOR-BASED CIRCUIT", &Font20, WHITE, BLACK);
    
    // Draw circuit specific name - centered in right box
    int subtitleX = TITLE_LEFT_WIDTH + (TITLE_LEFT_WIDTH / 2) - 100;
    Paint_DrawString_EN(subtitleX, TITLE_HEIGHT/2, names[nameIndex], &Font16, WHITE, BLACK);
    
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
    Point nodes[25];  // Max 25 nodes for safety
    int actualNumNodes = min(numNodes, 25);
    int nodesCreated = 0;
    
    // Helper function to add jitter to positions
    auto addJitter = [](int val, int jitterAmount) -> int {
        return val + random(-jitterAmount, jitterAmount);
    };
    
    // Create perimeter nodes to ensure full space usage
    // Top nodes
    for(int i = 0; i < 3 && nodesCreated < actualNumNodes; i++) {
        int x = circuitBounds.left + (i+1) * (circuitArea.width/4);
        int y = circuitBounds.top + 40;
        nodes[nodesCreated].x = x;
        nodes[nodesCreated].y = y;
        nodesCreated++;
    }
    
    // Bottom nodes
    for(int i = 0; i < 3 && nodesCreated < actualNumNodes; i++) {
        int x = circuitBounds.left + (i+1) * (circuitArea.width/4);
        int y = circuitBounds.bottom - 40;
        nodes[nodesCreated].x = x;
        nodes[nodesCreated].y = y;
        nodesCreated++;
    }
    
    // Left and right side nodes
    int availableHeight = circuitBounds.bottom - circuitBounds.top;
    for(int i = 0; i < 2 && nodesCreated < actualNumNodes; i++) {
        int y = circuitBounds.top + (i+1) * (availableHeight/3);
        
        // Left side
        nodes[nodesCreated].x = circuitBounds.left + 40;
        nodes[nodesCreated].y = y;
        nodesCreated++;
        
        if (nodesCreated < actualNumNodes) {
            // Right side
            nodes[nodesCreated].x = circuitBounds.right - 40;
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
        Display::circle(nodes[i].x, nodes[i].y, 5, true);
    }
    
    // Draw ground symbol at node 0
    Point groundNode = nodes[0];
    int groundX = groundNode.x;
    int groundY = groundNode.y + 30;
    Display::line(groundNode.x, groundNode.y, groundX, groundY);
    Display::line(groundX - 15, groundY, groundX + 15, groundY);
    Display::line(groundX - 10, groundY + 5, groundX + 10, groundY + 5);
    Display::line(groundX - 5, groundY + 10, groundX + 5, groundY + 10);
    
    // Add voltage source between node 1 and ground node
    int sourceX = (nodes[1].x + groundNode.x) / 2;
    int sourceY = (nodes[1].y + groundNode.y) / 2;
    int radius = 25;
    
    Display::circle(sourceX, sourceY, radius, false);
    Paint_DrawString_EN(sourceX - 5, sourceY - 8, "+", &Font16, WHITE, BLACK);
    Display::line(sourceX - 7, sourceY + 8, sourceX + 7, sourceY + 8);
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
        int width = 60;
        int height = 30;
        
        Display::rect(compX - width/2, compY - height/2, compX + width/2, compY + height/2, false);
        Paint_DrawString_EN(compX - 5, compY - 8, "M", &Font16, WHITE, BLACK);
        
        // Draw connecting lines
        Display::line(nodes[node1].x, nodes[node1].y, compX - width/2, compY);
        Display::line(nodes[node2].x, nodes[node2].y, compX + width/2, compY);
    }
    
    // Draw resistors
    for (int i = 0; i < numResistors; i++) {
        auto [node1, node2] = getRandomNodePair();
        int compX = (nodes[node1].x + nodes[node2].x) / 2;
        int compY = (nodes[node1].y + nodes[node2].y) / 2;
        int width = 50;
        int height = 20;
        
        // Draw zigzag resistor
        int startX = compX - width/2;
        int endX = compX + width/2;
        
        Display::line(nodes[node1].x, nodes[node1].y, startX, compY);
        Display::line(startX, compY, startX + 5, compY - height/2);
        Display::line(startX + 5, compY - height/2, startX + 15, compY + height/2);
        Display::line(startX + 15, compY + height/2, startX + 25, compY - height/2);
        Display::line(startX + 25, compY - height/2, startX + 35, compY + height/2);
        Display::line(startX + 35, compY + height/2, startX + 45, compY - height/2);
        Display::line(startX + 45, compY - height/2, endX, compY);
        Display::line(endX, compY, nodes[node2].x, nodes[node2].y);
    }
    
    // Draw capacitors
    for (int i = 0; i < numCapacitors; i++) {
        auto [node1, node2] = getRandomNodePair();
        int compX = (nodes[node1].x + nodes[node2].x) / 2;
        int compY = (nodes[node1].y + nodes[node2].y) / 2;
        int width = 40;
        int plateSpacing = 10;
        
        Display::line(nodes[node1].x, nodes[node1].y, compX - width/2, compY);
        Display::line(compX - width/2, compY - 15, compX - width/2, compY + 15);
        Display::line(compX - width/2 + plateSpacing, compY - 15, compX - width/2 + plateSpacing, compY + 15);
        Display::line(compX - width/2 + plateSpacing, compY, nodes[node2].x, nodes[node2].y);
    }
    
    // Draw inductors
    for (int i = 0; i < numInductors; i++) {
        auto [node1, node2] = getRandomNodePair();
        int compX = (nodes[node1].x + nodes[node2].x) / 2;
        int compY = (nodes[node1].y + nodes[node2].y) / 2;
        int width = 60;
        int loopRadius = 6;
        
        Display::line(nodes[node1].x, nodes[node1].y, compX - width/2, compY);
        
        // Draw loops
        for (int j = 0; j < 4; j++) {
            int loopX = compX - width/2 + 10 + j * 15;
            Display::circle(loopX, compY, loopRadius, false);
        }
        
        Display::line(compX + width/2 - 5, compY, nodes[node2].x, nodes[node2].y);
    }
    
    // Draw diodes
    for (int i = 0; i < numDiodes; i++) {
        auto [node1, node2] = getRandomNodePair();
        int compX = (nodes[node1].x + nodes[node2].x) / 2;
        int compY = (nodes[node1].y + nodes[node2].y) / 2;
        int width = 30;
        int height = 20;
        
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
    const int componentStartX = 30;
    const int componentStartY = CANVAS_HEIGHT - COMPONENTS_HEIGHT - FOOTER_HEIGHT + 30;
    
    char countText[50];
    sprintf(countText, "MEMRISTORS: %d", numMemristors);
    Paint_DrawString_EN(componentStartX, componentStartY, countText, &Font16, WHITE, BLACK);
    
    sprintf(countText, "RESISTORS: %d", numResistors);
    Paint_DrawString_EN(componentStartX, componentStartY + 20, countText, &Font16, WHITE, BLACK);
    
    sprintf(countText, "CAPACITORS: %d", numCapacitors);
    Paint_DrawString_EN(componentStartX, componentStartY + 40, countText, &Font16, WHITE, BLACK);
    
    sprintf(countText, "INDUCTORS: %d", numInductors);
    Paint_DrawString_EN(componentStartX, componentStartY + 60, countText, &Font16, WHITE, BLACK);
    
    sprintf(countText, "DIODES: %d", numDiodes);
    Paint_DrawString_EN(componentStartX, componentStartY + 80, countText, &Font16, WHITE, BLACK);
    
    // Add sample measurements in footer
    const int footerY = CANVAS_HEIGHT - FOOTER_HEIGHT + 40;
    
    unsigned long currentTime = millis() / 1000;
    sprintf(countText, "TIME: %lu s", currentTime);
    Paint_DrawString_EN(30, footerY, countText, &Font16, WHITE, BLACK);
    
    int sampleNum = random(100, 1000);
    sprintf(countText, "SAMPLE: %d", sampleNum);
    Paint_DrawString_EN(30, footerY + 30, countText, &Font16, WHITE, BLACK);
    
    Display::show();
}

// Function to draw the bridge display
void drawBridgeDisplay() {
    // Only clear the background on first display
    if (initialDisplay) {
        Display::background(WHITE);
        initialDisplay = false;
    }
    
    // Define tree positions and anchor points
    Point leftTrees[5];  // Store left bank tree positions
    Point rightTrees[5]; // Store right bank tree positions
    
    // Create tree positions
    for(int i = 0; i < 5; i++) {
        // Left bank trees
        leftTrees[i].x = 20 + i * 40;
        leftTrees[i].y = START_Y;
        
        // Right bank trees
        rightTrees[i].x = 670 + i * 40;
        rightTrees[i].y = START_Y;
    }
    
    // Draw banks - FIXED version
    // Left bank - specify end coordinates, not width/height
    Display::rect(0, START_Y, START_LEFT, START_Y + 600, true);    
    // Right bank - specify end coordinates, not width/height
    Display::rect(START_RIGHT, START_Y, EPD_13IN3K_WIDTH, START_Y + 600, true);
    
    // Draw trees on banks
    for(int i = 0; i < 5; i++) {
        drawTree(leftTrees[i].x, leftTrees[i].y, 80, -1.57, 5);
        drawTree(rightTrees[i].x, rightTrees[i].y, 80, -1.57, 5);
    }
    
    // Initialize roots with targets being trees on the opposite bank
    for(int i = 0; i < NUM_ROOTS; i++) {
        roots[i].numPoints = 1;
        roots[i].growing = true;
        roots[i].thickness = 3 + (i % 4);  // Increased thickness
        
        // Choose which tree this root will target (distribute roots among trees)
        int targetTreeIdx = i % 5;
        // Choose source tree with better distribution
        int sourceTreeIdx = (i / 2) % 5;  // Better distribution of starting trees
        
        if(i % 2 == 0) {
            // Left side roots starting from tree bases instead of bank edge
            roots[i].points[0] = {leftTrees[sourceTreeIdx].x, START_Y};  // Start at tree base
            roots[i].dirX = 0.95;  // Stronger horizontal direction
            // Make trajectories with more pronounced downward slope
            roots[i].dirY = ((i / 2) % 4 == 0) ? 0.12 : 0.22;  // Further increased downward bias with more diversity
            roots[i].targetX = rightTrees[targetTreeIdx].x;
            // Vary target height to make some roots climb up trees
            if ((i / 2) % 4 < 2) {
                // Higher up the tree
                roots[i].targetY = START_Y - 30 - (i * 8);  // More variety in target heights
            } else {
                // Near base of tree
                roots[i].targetY = START_Y - 10 - (i * 5);
            }
            roots[i].isLeft = true;
        } else {
            // Right side roots starting from tree bases instead of bank edge
            roots[i].points[0] = {rightTrees[sourceTreeIdx].x, START_Y};  // Start at tree base
            roots[i].dirX = -0.95;  // Stronger horizontal direction
            // Make trajectories with more pronounced downward slope
            roots[i].dirY = ((i / 2) % 4 == 1) ? 0.12 : 0.22;  // Further increased downward bias with more diversity
            roots[i].targetX = leftTrees[targetTreeIdx].x;
            // Vary target height to make some roots climb up trees
            if ((i / 2) % 4 < 2) {
                // Higher up the tree
                roots[i].targetY = START_Y - 30 - (i * 8);  // More variety in target heights
            } else {
                // Near base of tree
                roots[i].targetY = START_Y - 10 - (i * 5);
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
                // Left to right progress
                progressAcrossGap = (lastPoint->x - START_LEFT) / (float)(START_RIGHT - START_LEFT);
            } else {
                // Right to left progress
                progressAcrossGap = (START_RIGHT - lastPoint->x) / (float)(START_RIGHT - START_LEFT);
            }
            
            // Check if we're heading toward the middle or away from it
            bool headingTowardMiddle = false;
            if ((root->isLeft && progressAcrossGap < 0.5) || (!root->isLeft && progressAcrossGap < 0.5)) {
                headingTowardMiddle = true;
            }
            
            // Update direction - stronger influence as we get closer to tree
            float targetInfluence = min(0.5, 0.3 + (1 - distToTarget / 500) * 0.3);
            float newDirX = (1 - targetInfluence) * root->dirX + targetInfluence * (dx / distToTarget);
            float newDirY = (1 - targetInfluence) * root->dirY + targetInfluence * (dy / distToTarget);
            
            // Add varied bias based on root index for diversity
            // Only add bias in the middle of trajectory
            if (distToTarget > 100 && distToTarget < 400) {
                // Apply stronger downward bias in the middle of the gap
                float midSectionModifier = 1.0;
                if (progressAcrossGap > 0.3 && progressAcrossGap < 0.7) {
                    // Additional downward force in the middle section - much stronger gravity effect
                    midSectionModifier = 3.0;
                    
                    // Add extra downward bias regardless of root type - significantly increased
                    newDirY += 0.3 * midSectionModifier;
                    
                    // Ensure newDirY is ALWAYS positive in the middle (downward)
                    if (newDirY < 0) {
                        newDirY = 0.2; // Force at least a slight downward direction
                    }
                } else {
                    if (i % 4 < 2) {
                        // More pronounced downward bias
                        newDirY += 0.09;
                    } else {
                        // Increased curve with stronger downward bias
                        newDirY += 0.12;
                    }
                    
                    // When heading toward the middle, never allow upward movement
                    if (headingTowardMiddle && newDirY < 0) {
                        newDirY = 0.05; // Force at least a slight downward direction
                    }
                }
            }
            
            // When getting close to target tree, aim upward to climb it
            if (distToTarget < 100) {
                // Only pull upward if we're near the end, not in the middle section
                // AND not heading toward the middle
                if (progressAcrossGap > 0.75 && !headingTowardMiddle) {
                    // Pull upward to climb the tree, but only when very close to the target tree
                    newDirY -= 0.1;
                }
            }
            
            // Add random variation (reduced for straighter paths)
            newDirX += randomFloat(-0.02, 0.02);
            float randY = randomFloat(-0.02, 0.02);
            
            // Ensure random variation doesn't cause upward movement when heading to the middle
            if (headingTowardMiddle || (progressAcrossGap > 0.3 && progressAcrossGap < 0.7)) {
                // Only allow downward random variation
                randY = randomFloat(0, 0.03);
            }
            newDirY += randY;
            
            // Normalize direction
            float len = sqrt(newDirX * newDirX + newDirY * newDirY);
            root->dirX = newDirX / len;
            root->dirY = newDirY / len;
            
            // Final safety check - ensure no upward movement when heading to middle
            // or in the middle section
            if ((headingTowardMiddle || progressAcrossGap > 0.3 && progressAcrossGap < 0.7) && root->dirY < 0) {
                root->dirY = abs(root->dirY) * 0.5; // Convert any negative Y direction to positive
            }
            
            // Calculate new point
            Point newPoint = {
                (int16_t)(lastPoint->x + root->dirX * 15),
                (int16_t)(lastPoint->y + root->dirY * 15)
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
            
            // Draw root segment with increased thickness
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
            
            if(distToTree < 20) {
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
                    
                    if(dist < 60) {  // Further increased connection distance for better connectivity
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
                   newPoint.y < 100 || newPoint.y > START_Y + 350) {  // Adjusted bounds
                    root->growing = false;
                }
            }
        }
    }
    
    // Add hanging roots - longer, thicker, and more frequent
    for(int i = 0; i < NUM_ROOTS; i++) {
        for(int j = 1; j < roots[i].numPoints; j += 2) {  // Even more frequent hanging roots (every other point)
            // Calculate progress across gap
            float progressAcrossGap = 0.0;
            if (roots[i].isLeft) {
                // Left to right progress
                progressAcrossGap = (roots[i].points[j].x - START_LEFT) / (float)(START_RIGHT - START_LEFT);
            } else {
                // Right to left progress
                progressAcrossGap = (START_RIGHT - roots[i].points[j].x) / (float)(START_RIGHT - START_LEFT);
            }
            
            // Higher probability and longer hangers in the middle
            float hangingProbability = (progressAcrossGap > 0.3 && progressAcrossGap < 0.7) ? 0.65 : 0.5;
            
            if(randomFloat(0, 1) < hangingProbability) {  // Increased probability
                Point* point = &roots[i].points[j];
                
                // Base hanging length, longer in the middle
                int hangLen = random(20, 40);  // Longer hangers
                
                // Extra length in the middle - even longer for more dramatic effect
                if (progressAcrossGap > 0.3 && progressAcrossGap < 0.7) {
                    hangLen += random(20, 35);  // Much longer extra length in middle for gravity effect
                }
                
                int hangX = random(-5, 5);
                for(int t = 0; t < 3; t++) {  // Thicker hangers
                    Display::line(point->x, point->y,
                                point->x + hangX, point->y + hangLen + t);
                }
            }
        }
    }
    
    // Add climbing tendrils on trees
    const int tendrilLength = 20;
    
    // Left tree tendrils
    for(int i = 0; i < 5; i++) {
        for(int j = 0; j < 2 + random(0, 2); j++) {  // 2-3 tendrils per tree
            int startY = leftTrees[i].y - random(10, 40);
            int endY = startY - random(15, 30);
            
            // Create wavy pattern
            int midX1 = leftTrees[i].x + random(5, 10);
            int midX2 = leftTrees[i].x - random(5, 10);
            
            Display::line(leftTrees[i].x, startY, midX1, startY - tendrilLength/3);
            Display::line(midX1, startY - tendrilLength/3, midX2, startY - tendrilLength*2/3);
            Display::line(midX2, startY - tendrilLength*2/3, leftTrees[i].x, endY);
        }
    }
    
    // Right tree tendrils
    for(int i = 0; i < 5; i++) {
        for(int j = 0; j < 2 + random(0, 2); j++) {  // 2-3 tendrils per tree
            int startY = rightTrees[i].y - random(10, 40);
            int endY = startY - random(15, 30);
            
            // Create wavy pattern
            int midX1 = rightTrees[i].x - random(5, 10);
            int midX2 = rightTrees[i].x + random(5, 10);
            
            Display::line(rightTrees[i].x, startY, midX1, startY - tendrilLength/3);
            Display::line(midX1, startY - tendrilLength/3, midX2, startY - tendrilLength*2/3);
            Display::line(midX2, startY - tendrilLength*2/3, rightTrees[i].x, endY);
        }
    }
    
    // Add a new section after the main growth loop to ensure all roots are connected
    // Ensure all roots connect - create connections for any unconnected roots
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
            if(closestIdx >= 0 && minDist < 200) {
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
                    int midY = (lastPoint->y + connPoint->y) / 2 + random(-20, 20);
                    
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
    
    // Draw based on current mode
    if (currentMode == BRIDGE_MODE) {
      Serial.println("Drawing indigenous bridge...");
      drawBridgeDisplay();
    } else {
      Serial.println("Drawing chaotic circuit...");
      drawChaoticCircuit();
    }
    
    displayUpdateRequested = false;
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
