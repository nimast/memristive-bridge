/* Includes ------------------------------------------------------------------*/
#include "DEV_Config.h"
#include "EPD.h"
#include "GUI_Paint.h"
#include "imagedata.h"
#include <stdlib.h>
#include "Display.h"

/* Entry point ----------------------------------------------------------------*/
void setup()
{
    DEV_Module_Init();
    Display::begin();
      
    // Simple drawing example
    Display::background(WHITE);
    
    // Draw a house
    Display::rect(100, 100, 200, 150, false);    // house frame
    Display::rect(150, 160, 50, 90, false);      // door
    Display::rect(270, 130, 40, 40, false);      // window
    
    // Roof (triangle using lines)
    Display::line(100, 100, 200, 50);
    Display::line(200, 50, 300, 100);
    
    // Add some text
    Display::text("My House", 150, 220, true);
    
    // Show the drawing
    Display::show();
    delay(3000);
    
    Display::end();
}

/* The main loop -------------------------------------------------------------*/
void loop()
{
  // Nothing to do here
}
