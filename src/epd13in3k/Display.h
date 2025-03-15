#ifndef DISPLAY_H
#define DISPLAY_H

#include "GUI_Paint.h"

class Display {
public:
    // Initialize the display
    static void begin() {
        EPD_13IN3K_Init();
        EPD_13IN3K_Clear();
        
        // Allocate image buffer
        imageSize = ((EPD_13IN3K_WIDTH % 8 == 0) ? 
            (EPD_13IN3K_WIDTH / 8) : (EPD_13IN3K_WIDTH / 8 + 1)) * EPD_13IN3K_HEIGHT;
        buffer = (UBYTE *)malloc(imageSize);
        
        if (buffer == NULL) {
            Serial.println("Failed to allocate memory");
            while(1);
        }

        // Initialize Paint
        Paint_NewImage(buffer, EPD_13IN3K_WIDTH, EPD_13IN3K_HEIGHT, 0, WHITE);
        Paint_SelectImage(buffer);
        Paint_Clear(WHITE);
    }

    // Basic drawing functions with simpler signatures
    static void background(UWORD color) {
        Paint_Clear(color);
    }
    
    static void point(int x, int y) {
        Paint_DrawPoint(x, y, BLACK, DOT_PIXEL_1X1, DOT_STYLE_DFT);
    }
    
    static void line(int x1, int y1, int x2, int y2) {
        Paint_DrawLine(x1, y1, x2, y2, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    }
    
    static void rect(int x1, int y1, int x2, int y2, bool filled = false) {
        Paint_DrawRectangle(x1, y1, x2, y2, BLACK, DOT_PIXEL_1X1, 
            filled ? DRAW_FILL_FULL : DRAW_FILL_EMPTY);
    }
    
    static void circle(int x, int y, int radius, bool filled = false) {
        Paint_DrawCircle(x, y, radius, BLACK, DOT_PIXEL_1X1, 
            filled ? DRAW_FILL_FULL : DRAW_FILL_EMPTY);
    }
    
    static void text(const char* str, int x, int y, bool large = false) {
        Paint_DrawString_EN(x, y, str, 
            large ? &Font16 : &Font12, BLACK, WHITE);
    }

    // Update display
    static void show() {
        EPD_13IN3K_Display(buffer);
    }

    // Clean up
    static void end() {
        EPD_13IN3K_Clear();
        EPD_13IN3K_Sleep();
        free(buffer);
        buffer = NULL;
    }

    static void drawRootLine(int x1, int y1, int x2, int y2, int thickness = 1) {
        for(int i = 0; i < thickness; i++) {
            Paint_DrawLine(x1, y1+i, x2, y2+i, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
        }
    }

    static void bezier(int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4) {
        // Simple bezier curve implementation
        for(float t = 0; t <= 1; t += 0.02) {
            float t1 = (1-t);
            float px = t1*t1*t1*x1 + 3*t1*t1*t*x2 + 3*t1*t*t*x3 + t*t*t*x4;
            float py = t1*t1*t1*y1 + 3*t1*t1*t*y2 + 3*t1*t*t*y3 + t*t*t*y4;
            
            if(t > 0) {
                Paint_DrawLine(prevX, prevY, px, py, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
            }
            prevX = px;
            prevY = py;
        }
    }

private:
    static UBYTE* buffer;
    static UDOUBLE imageSize;
    static float prevX, prevY;  // Add these as class members
};

// Define static members
UBYTE* Display::buffer = NULL;
UDOUBLE Display::imageSize = 0;

#endif 