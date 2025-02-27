// If not using a Feather, you might need to comment out one or both of these lines:
#define FEATHER_M4_EXPRESS  // if using the M4 Express board
#define USE_USBCON          // NEEDED FOR CPU with Built-in USB.  ATmega32u4 - Feather 32u4, Feather M4, LEONARDO...

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

#include <Wire.h>


// Optional DMA for Neopixels (to prevent conflict with serial port)
#include <Adafruit_NeoPixel_ZeroDMA.h>


#define NEOPIXEL_STRIP_PIN MOSI  // Pin M0 on board, Supports DMA
#define NEOPIXEL_STRIP_LENGTH 8



// Use DMA NeoPixel Library
Adafruit_NeoPixel_ZeroDMA Neopixel_Strip = Adafruit_NeoPixel_ZeroDMA(NEOPIXEL_STRIP_LENGTH, NEOPIXEL_STRIP_PIN, NEO_GRB + NEO_KHZ800);

#ifdef FEATHER_M4_EXPRESS
// enable the single onboard Neopixel
#define NUMBER_OF_ONBOARD_NEOPIXELS 1
#define ONBOARD_NEOPIXEL_PIN 8
#define ONBOARD_NEOPIXEL_INDEX 0
Adafruit_NeoPixel onboard_neopixel = Adafruit_NeoPixel(NUMBER_OF_ONBOARD_NEOPIXELS, ONBOARD_NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
#endif

int button_number = 0;

////////////////////////////////////////////////////////////////////////////////
void setup() {

  pinMode(LED_BUILTIN, OUTPUT);  // heartbeat LED
  // randomSeed(analogRead(0));
#ifdef FEATHER_M4_EXPRESS  // Feather M4 has a neopixel we can blink
  onboard_neopixel.begin();
  onboard_neopixel.setPixelColor(ONBOARD_NEOPIXEL_INDEX, 63, 63, 63);  // pixel index, color
  onboard_neopixel.show();
#endif


  // blink LED on the board at startup
  for (int i = 0; i < 10; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }

  Neopixel_Strip.begin();
  Neopixel_Strip.show();  // Initialize all pixels to 'off'


  // Test Neopixel strip
  neopixel_strip_solid(Neopixel_Strip.Color(20, 0, 0));  //GBR
  delay(500);
  neopixel_strip_solid(Neopixel_Strip.Color(0, 20, 0));  //GBR
  delay(500);
  neopixel_strip_solid(Neopixel_Strip.Color(0, 0, 20));  //GBR
  delay(500);

  neopixel_strip_solid(Neopixel_Strip.Color(0, 0, 0));  //GBR - OFF

  Serial1.begin(115200);  // bluetooth connection with Android phone
}


////////////////////////////////////////////////////////////////////////////////
void loop() {

#ifdef FEATHER_M4_EXPRESS
  onboard_neopixel.setPixelColor(ONBOARD_NEOPIXEL_INDEX, 63, 0, 0);  // Set pixel to Red before trying I2C IMU
  onboard_neopixel.show();
#endif

  // blink strip led
  Neopixel_Strip.setPixelColor(1, Neopixel_Strip.Color(20, 0, 0));
  Neopixel_Strip.show();
  delay(200);

  // Onboard Neopixel Heartbeat
#ifdef FEATHER_M4_EXPRESS
  onboard_neopixel.setPixelColor(ONBOARD_NEOPIXEL_INDEX, 0, 0, 63);  // Set pixel to blue for battery and sleep time
  onboard_neopixel.show();
#endif



  if (Serial1.available() > 0) {  // Checks whether data is comming from the serial port
    button_number = Serial1.read();       // Reads the data from the serial port

    if (button_number == 0) {
      // Read Accelerometer data 

      int acc_x = Serial1.read();
      Serial.print("X = ");
      Serial.println(acc_x, DEC); // Arduino local console       
      int acc_y = Serial1.read();
      Serial.print("Y = ");
      Serial.println(acc_y, DEC); // Arduino local console       
      int acc_z = Serial1.read();
      Serial.print("Z = ");
      Serial.println(acc_y, DEC); // Arduino local console       


    } else if (button_number == 1) {
      Neopixel_Strip.setPixelColor(3, Neopixel_Strip.Color(0, 0, 20));  // Blue
      Neopixel_Strip.show();
      Serial1.print("LED: BLUE");  // Send back, to the phone, the String "LED: ON"

    } else if (button_number == 2) {
      Neopixel_Strip.setPixelColor(3, Neopixel_Strip.Color(0, 20, 0));  // Green
      Neopixel_Strip.show();
      Serial1.print("LED: GREEN");

    }
    else {
      Serial1.print("Received: " + String(button_number)); // to the phone

    }
    Serial.print("Received Button: "); // Arduino local console
    Serial.println(button_number, DEC); // Arduino local console
    //Serial.println("Received Button: " + String(button_number)); // Arduino local console
  }

  //delay(200);


  Neopixel_Strip.setPixelColor(1, Neopixel_Strip.Color(7, 7, 7));  // White
  Neopixel_Strip.show();
  delay(200);
}

void neopixel_strip_solid(uint32_t c) {
  for (uint16_t i = 0; i < Neopixel_Strip.numPixels(); i++) {
    Neopixel_Strip.setPixelColor(i, c);
  }
  Neopixel_Strip.show();
}
