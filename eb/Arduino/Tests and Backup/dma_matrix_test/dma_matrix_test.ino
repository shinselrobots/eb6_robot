// Adafruit_NeoMatrix example for single NeoPixel Shield.
// Scrolls 'Howdy' across the matrix in a portrait (vertical) orientation.

#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix_ZeroDMA.h>

// NOTE: matrix shield must be REWIRED for a different pin.  By default it
// works with Arduino pin 6, but that pin is NOT COMPATIBLE with the
// NeoPixel_ZeroDMA library -- DMA NeoPixels work ONLY on SPECIFIC PINS.
// On Circuit Playground Express: 8, A2 and A7 (TX) are valid.
// On Feather M0, Arduino Zero, etc.: 5, 11, A5 and 23 (SPI MOSI).
// On GEMMA M0: pin 0.
#define MATRIX1_PIN 12
#define PIN MOSI
#define MATRIX_ROWS 4
#define MATRIX_COLUMNS 8

// MATRIX DECLARATION:
// Parameter 1 = width of NeoPixel matrix
// Parameter 2 = height of matrix
// Parameter 3 = pin number (most are valid)
// Parameter 4 = matrix layout flags, add together as needed:
//   NEO_MATRIX_TOP, NEO_MATRIX_BOTTOM, NEO_MATRIX_LEFT, NEO_MATRIX_RIGHT:
//     Position of the FIRST LED in the matrix; pick two, e.g.
//     NEO_MATRIX_TOP + NEO_MATRIX_LEFT for the top-left corner.
//   NEO_MATRIX_ROWS, NEO_MATRIX_COLUMNS: LEDs are arranged in horizontal
//     rows or in vertical columns, respectively; pick one or the other.
//   NEO_MATRIX_PROGRESSIVE, NEO_MATRIX_ZIGZAG: all rows/columns proceed
//     in the same order, or alternate lines reverse direction; pick one.
//   See example below for these values in action.
// Parameter 5 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_GRBW    Pixels are wired for GRBW bitstream (RGB+W NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)


// Example for NeoPixel Shield.  In this application we'd like to use it
// as a 5x8 tall matrix, with the USB port positioned at the top of the
// Arduino.  When held that way, the first pixel is at the top right, and
// lines are arranged in columns, progressive order.  The shield uses
// 800 KHz (v2) pixels that expect GRB color data.
Adafruit_NeoMatrix_ZeroDMA matrix1(MATRIX_ROWS, MATRIX_COLUMNS, MATRIX1_PIN,
  NEO_MATRIX_COLUMNS + NEO_MATRIX_PROGRESSIVE,
  NEO_GRB);

Adafruit_NeoMatrix_ZeroDMA matrix(MATRIX_ROWS, MATRIX_COLUMNS, PIN,
  NEO_MATRIX_TOP     + NEO_MATRIX_RIGHT +
  NEO_MATRIX_COLUMNS + NEO_MATRIX_PROGRESSIVE,
  NEO_GRB);

const uint16_t colors[] = {
  matrix.Color(255, 0, 0), matrix.Color(0, 255, 0), matrix.Color(0, 0, 255), matrix.Color(0, 0, 0) };

void setup() {
  randomSeed(analogRead(0));
  matrix1.begin();
  matrix.begin();
  matrix1.setTextWrap(false);
  matrix.setTextWrap(false);
  matrix1.setTextColor(colors[0]);
  matrix.setTextColor(colors[0]);
  matrix1.setBrightness(8);
  matrix.setBrightness(8);

}

int x    = matrix.width();
int pass = 0;
bool print_text = false;
int global_color = 0xFF;

void loop() {
  matrix1.fillScreen(0);
  matrix.fillScreen(0);
  matrix1.setCursor(x, 0);
  matrix.setCursor(x, 0);

  if(print_text) {
    matrix.print(F("Howdy"));
    if(--x < -36) {
      x = matrix.width();
      if(++pass >= 3) pass = 0;
      matrix.setTextColor(colors[pass]);
    }
    matrix.show();
    delay(100);
  }
  else {
    
    for(int x=0; x < MATRIX_COLUMNS; x++) {
      for(int y=0; y < MATRIX_ROWS; y++) {
        int16_t randColor = 0; // default to off
        int randOn = random(0, 10);
        if(randOn < 5) {
          randColor = global_color;
        }
        matrix1.drawPixel(y,x, randColor);
        matrix.drawPixel(y,x, randColor);
      }
    }
    matrix1.show();
    matrix.show();
    int rand_delay = random(50, 300);
    delay(rand_delay);
  }
}
