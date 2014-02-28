#include <Adafruit_NeoPixel.h>

#define PIN 6

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(30, PIN, NEO_GRB + NEO_KHZ800);


#include <CapacitiveSensor.h>

/*
 analog-plot
 
 Read analog values from A0 and A1 and print them to serial port.

 Modified from an original by electronut.in

*/

#include "Arduino.h"

CapacitiveSensor   cs_3_2 = CapacitiveSensor(3,2);        // 10 megohm resistor between pins 3 & 2, pin 2 is sensor pin, add wire, foil
CapacitiveSensor   cs_9_8 = CapacitiveSensor(9,8);        // 10 megohm resistor between pins 9 & 8, pin 8 is sensor pin, add wire, foil

void setup() {
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
// initialize serial comms
  Serial.begin(9600); 
}

float scale_a=0.1;
float scale_b=0.1;
float location;
long start;
float total1, total2;
float norm_a, norm_b;

void loop()
{
 
 total1 =  cs_3_2.capacitiveSensor(30);
 total2 =  cs_9_8.capacitiveSensor(30);
  
 if( total1> scale_a) {
   scale_a = total1;
 }
 if( total2> scale_b) {
   scale_b = total2;
 }
 
 norm_a = total1/scale_a;
 norm_b = total2/scale_b;
 
 location = ((1-norm_a)+norm_b)/2;
   // print to serial
 // Serial.print((total1/NORM_SCALE_A)*1000.0);
  Serial.print(scale_a);
  Serial.print(" ");
  Serial.print(total1);
  Serial.print(" ");
  Serial.print(scale_b);
  Serial.print(" ");
  Serial.print(total2);
  Serial.print(" ");
  Serial.print(location);
  Serial.print("\n");
  if(norm_a > 0.1 || norm_b > 0.1) {
    color( HueToRGB((location)*360.0)   ); // Red
  } else {
    color(strip.Color(0, 0, 0)); // off/black
  }
}

// Fill the dots one after the other with a color
void color(uint32_t c) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
  }
}


// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

uint32_t rgb_to_colour(double r, double g, double b) {
 return strip.Color(r*255, g*255, b*255); 
}
uint32_t HueToRGB(double h)
{
    uint32_t color;
    double v = 1.0;
    double s = 1.0;
    double c = 0.0, m = 0.0, x = 0.0;
    
        if (h < 0) {
          h = 0;
        }
        c = v * s;
        x = c * (1.0 - fabs(fmod(h / 60.0, 2) - 1.0));
        m = v - c;
        if (h >= 0.0 && h < 60.0)
        {
            color = rgb_to_colour(c + m, x + m, m);
        }
        else if (h >= 60.0 && h < 120.0)
        {
            color = rgb_to_colour(x + m, c + m, m);
        }
        else if (h >= 120.0 && h < 180.0)
        {
            color = rgb_to_colour(m, c + m, x + m);
        }
        else if (h >= 180.0 && h < 240.0)
        {
            color = rgb_to_colour(m, x + m, c + m);
        }
        else if (h >= 240.0 && h < 300.0)
        {
            color = rgb_to_colour(x + m, m, c + m);
        }
        else if (h >= 300.0 && h < 360.0)
        {
            color = rgb_to_colour(c + m, m, x + m);
        }
        else
        {
            color = rgb_to_colour(m, m, m);
        }
    
    return color;
}
