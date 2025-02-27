// NeoPixel test program showing use of the WHITE channel for RGBW
// pixels only (won't look correct on regular RGB NeoPixel strips).

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1:
#define EYE_LED_PIN     6
#define EAR_LED_PIN     9

// How many NeoPixels are attached to the Arduino?
#define LED_COUNT  32

// NeoPixel brightness, 0 (min) to 255 (max)
#define BRIGHTNESS 50 // Set BRIGHTNESS to about 1/5 (max = 255)

// Declare our NeoPixel strip object:
Adafruit_NeoPixel EyeStrip(LED_COUNT, EYE_LED_PIN, NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel EarStrip(LED_COUNT, EAR_LED_PIN, NEO_GRB + NEO_KHZ800);
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)

void setup() {
  // These lines are specifically to support the Adafruit Trinket 5V 16 MHz.
  // Any other board, you can remove this part (but no harm leaving it):
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
  // END of Trinket-specific code.

  EyeStrip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  EarStrip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)

  EyeStrip.show();            // Turn OFF all pixels ASAP
  EarStrip.show();            // Turn OFF all pixels ASAP

  EyeStrip.setBrightness(BRIGHTNESS);
  EarStrip.setBrightness(BRIGHTNESS);

  // Light top two pixels of each eye and ear (for istallaton)
  EyeStrip.setPixelColor(0, EyeStrip.Color(0, 0, 0, 155));
  EyeStrip.setPixelColor(15, EyeStrip.Color(0, 0, 0, 155));
  EyeStrip.setPixelColor(16, EyeStrip.Color(0, 0, 0, 155));
  EyeStrip.setPixelColor(31, EyeStrip.Color(0, 0, 0, 155));
  EyeStrip.show();            // 
 
  EarStrip.setPixelColor(0, EarStrip.Color(0, 150, 150));
  EarStrip.setPixelColor(15, EarStrip.Color(0, 150, 150));
  EarStrip.setPixelColor(16, EarStrip.Color(0, 150, 150));
  EarStrip.setPixelColor(31, EarStrip.Color(0, 150, 150));
  EarStrip.show();            // 
  
  delay(500);
 
  EyeColorWipe(EyeStrip.Color(  50,   50,    50, 150), 10); // True white or RGB white
  EyeStrip.show();            // 

  EarColorWipe(EarStrip.Color(  50,   50,    50), 10); // RGB white
  EarStrip.show();            // 

}

void loop() {
  // Fill along the length of the strip in various colors...

  //EarColorWipe(EarStrip.Color(255,   0,   0), 10); // Red
  //pulseWhite(3);
  //EarColorWipe(EarStrip.Color(  0, 255,   0), 10); // Green
  //pulseWhite(3);
  //EarColorWipe(EarStrip.Color(  0,   0, 255), 10); // Blue
  //pulseWhite(3);

//  EyeColorWipe(EyeStrip.Color(  0,   0,   0, 255), 50); // True white (not RGB white)
  EarRainbow(2);
 
 ReversePulseWhite(0);

  //pulseWhite(5);
  delay(1);

//  rainbowFade2White(3, 3, 1);
}

// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a delay time (in milliseconds) between pixels.
void EyeColorWipe(uint32_t color, int wait) {
  for(int i=0; i<EyeStrip.numPixels(); i++) { // For each pixel in EyeStrip...
    EyeStrip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    EyeStrip.show();                          //  Update EyeStrip to match
    delay(wait);                           //  Pause for a moment
  }
}

void EarColorWipe(uint32_t color, int wait) {
  for(int i=0; i<EarStrip.numPixels(); i++) { // For each pixel in EarStrip...
    EarStrip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    EarStrip.show();                          //  Update EarStrip to match
    delay(wait);                           //  Pause for a moment
  }
}


void pulseWhite(uint8_t wait) {
  for(int j=0; j<256; j++) { // Ramp up from 0 to 255
    // Fill entire EyeStrip with white at gamma-corrected brightness level 'j':
    EyeStrip.fill(EyeStrip.Color(0, 0, 0, EyeStrip.gamma8(j)));
    EyeStrip.show();
    delay(wait);
  }

  for(int j=255; j>=0; j--) { // Ramp down from 255 to 0
    EyeStrip.fill(EyeStrip.Color(0, 0, 0, EyeStrip.gamma8(j)));
    EyeStrip.show();
    delay(wait);
  }
}

void ReversePulseWhite(uint8_t wait) {

  for(int j=155; j>=0; j--) { // Ramp down from 255 to 0
    EyeStrip.fill(EyeStrip.Color(0, 0, 0, EyeStrip.gamma8(j)));
    EyeStrip.show();
    delay(wait);
  }

  for(int j=0; j<156; j++) { // Ramp up from 0 to 255
    // Fill entire EyeStrip with white at gamma-corrected brightness level 'j':
    EyeStrip.fill(EyeStrip.Color(0, 0, 0, EyeStrip.gamma8(j)));
    EyeStrip.show();
    delay(wait);
  }

}

// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void EarRainbow(int wait) {
  // Hue of first pixel runs 5 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
  // means we'll make 5*65536/256 = 1280 passes through this loop:
  for(long firstPixelHue = 0; firstPixelHue < 5*65536; firstPixelHue += 256) {
    // strip.rainbow() can take a single argument (first pixel hue) or
    // optionally a few extras: number of rainbow repetitions (default 1),
    // saturation and value (brightness) (both 0-255, similar to the
    // ColorHSV() function, default 255), and a true/false flag for whether
    // to apply gamma correction to provide 'truer' colors (default true).
    EarStrip.rainbow(firstPixelHue);
    // Above line is equivalent to:
    // strip.rainbow(firstPixelHue, 1, 255, 255, true);
    EarStrip.show(); // Update strip with new contents
    delay(wait);  // Pause for a moment
  }
}
