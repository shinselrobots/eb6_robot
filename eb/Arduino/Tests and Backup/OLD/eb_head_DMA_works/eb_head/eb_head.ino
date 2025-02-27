// EB Head controls Eye and Ear lights, and maybe sensors in the future

// Adafruit NeoPixel used for the eyes
// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

// Optional DMA for Neopixels (to prevent conflict with serial port)
#include <Adafruit_NeoPixel_ZeroDMA.h>

// If not using a Feather, you might need to comment out one or both of these lines:
#define FEATHER_M4_EXPRESS     // if using the M4 Express board
#define USE_USBCON  // NEEDED FOR CPU with Built-in USB.  ATmega32u4 - Feather 32u4, Feather M4, LEONARDO...



// ROS
#include <ros.h>
//#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
//#include <std_msgs/Empty.h>
//#include <geometry_msgs/Twist.h>
//#include <sensor_msgs/Joy.h>
//#include <behavior_common/CommandState.h>
#include <ros/time.h>
//#include "RobotConstants.h"

// Structures
typedef struct 
 {
     uint8_t g;
     uint8_t r;
     uint8_t b;
     uint8_t w;
 } EyeColor_t;

typedef struct 
 {
     uint8_t g;
     uint8_t r;
     uint8_t b;
 } EarColor_t;



/////////////////////////////////////////////////////////////////////////////////////
// Constants

#define NeoEyesPIN A4 //6
#define NeoEarsPIN A2 //9 
#define PIXELS_PER_RING 16
#define DEFAULT_EYE_BRIGHTNESS 50 // med-low brightness
#define DEFAULT_EAR_BRIGHTNESS 50 // med-low brightness
#define FADE_MODE_EAR_BRIGHTNESS 50 // med-low brightness
const uint32_t BLACK = 0;
const int BRIGHTNESS =  50;
const int WHITE_LEVEL = BRIGHTNESS;
const int RED_LEVEL =   BRIGHTNESS;
const int GREEN_LEVEL = BRIGHTNESS;
const int BLUE_LEVEL =  BRIGHTNESS;


// Parameter 1 = number of pixels in eyesStrip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)

enum EYE_CMD_STATE_T {
  EYES_OFF = 0,
  EYES_ON_SOLID,
  EYES_AUTO_BLINK,
};


/////////////////////////////////////////////////////////////////////////////////////
// Global Variables

EYE_CMD_STATE_T EyeCmdState = EYES_OFF;
uint8_t         EarCmdMode = 0; // TODO 0;
bool            HeartBeatLedState = false;
bool            FadeOnState = false;
uint32_t        onboardNeoPixelColor = 0;
const uint8_t   onboardNeoPixelIntensity = 32; // 0 - 255


//Adafruit_NeoPixel eyesStrip = Adafruit_NeoPixel((PIXELS_PER_RING * 2), NeoEyesPIN, NEO_GRBW + NEO_KHZ800);
//Adafruit_NeoPixel earsStrip = Adafruit_NeoPixel((PIXELS_PER_RING * 2), NeoEarsPIN, NEO_GRB + NEO_KHZ800);

Adafruit_NeoPixel_ZeroDMA eyesStrip = Adafruit_NeoPixel_ZeroDMA((PIXELS_PER_RING * 2), NeoEyesPIN, NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel_ZeroDMA earsStrip = Adafruit_NeoPixel_ZeroDMA((PIXELS_PER_RING * 2), NeoEarsPIN, NEO_GRB + NEO_KHZ800);


#ifdef FEATHER_M4_EXPRESS
   // enable the single onboard Neopixel
#define NUMBER_OF_ONBOARD_NEOPIXELS 1
#define ONBOARD_NEOPIXEL_PIN 8  
Adafruit_NeoPixel onboard_neopixel = Adafruit_NeoPixel(NUMBER_OF_ONBOARD_NEOPIXELS, ONBOARD_NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
#endif


EyeColor_t eyeColor = {0, 0, 0, 0}; // NOTE! Eyes are RGB + White!
EarColor_t earColor = {0, 0, 0};    

bool blinkNow = false; // force wait loop to exit immediately
ros::NodeHandle   nh;


// ROS node subscribers and publishers

// EYE MODE CMD: To test, try this: rostopic pub -1 /head/eye_cmd std_msgs/UInt16  2 // Eyes Blink
void eye_cmd_callback(const std_msgs::UInt16& cmd_msg) {
  if(0 == cmd_msg.data) {
    nh.loginfo("Head Arduino: EYE COMMAND = OFF");
    EyeCmdState = EYES_OFF;
    EyesOff();
  }
  else if(1 == cmd_msg.data) {
    nh.loginfo("Head Arduino: EYE COMMAND = ON");
    EyeCmdState = EYES_ON_SOLID;
    EyesOn(eyesStrip.Color(eyeColor.r, eyeColor.g, eyeColor.b, eyeColor.w));
  }
  else {
    nh.loginfo("Head Arduino: EYE COMMAND = BLINK");
    EyeCmdState = EYES_AUTO_BLINK;
  }
}
ros::Subscriber<std_msgs::UInt16> eyeCommandSubscriber("/head/eye_cmd", &eye_cmd_callback);

// EYE COLOR: To test, try this: rostopic pub -1 /head/eye_color std_msgs/UInt32 0x2f2f2f (dim white)
void eye_color_callback(const std_msgs::UInt32& cmd_msg) {
  uint32_t color = cmd_msg.data;

  if( 0x00002f == color) {
    nh.loginfo("Head Arduino Got EYE COLOR message: DEFAULT COLOR (BLUE)");
  }
  else {
    nh.loginfo("Head Arduino Got EYE COLOR message: CUSTOM COLOR");
  }
  eyeColor.b = color & 0xFF;
  eyeColor.g = (color >> 8) & 0xFF;
  eyeColor.r = (color >> 16) & 0xFF;
  eyeColor.w = (color >> 24) & 0xFF;
  blinkNow = true; // force wait loop to exit immediately
}
ros::Subscriber<std_msgs::UInt32> eyeColorSubscriber("/head/eye_color", &eye_color_callback);


// EAR MODE CMD: Set the ears light pattern. To test, try this: rostopic pub -1 /head/ear_cmd std_msgs/UInt16  1 // Ears On, default mode
void ear_cmd_callback(const std_msgs::UInt16& cmd_msg) {
  EarCmdMode = cmd_msg.data;

  // NOTE: Actual Mode selection is handled later!
  if(0 == EarCmdMode) {
    nh.loginfo("Head Arduino: EAR COMMAND = OFF");
  }
  else if(EarCmdMode > 2) {
    nh.loginfo("Head Arduino: EAR COMMAND  - UNKNOWN MODE!");
  }
  else {  
    nh.loginfo("Head Arduino: EAR COMMAND - KNOWN MODE");
  }

}
ros::Subscriber<std_msgs::UInt16> earCommandSubscriber("/head/ear_cmd", &ear_cmd_callback);


// EAR COLOR: To test, try this: rostopic pub -1 /head/ear_color std_msgs/UInt32 "0x002f00" (green)
void ear_color_callback(const std_msgs::UInt32& cmd_msg) {
  uint32_t color = cmd_msg.data;

  nh.loginfo("Head Arduino Got EAR COLOR message");
  earColor.b = color & 0xFF;
  earColor.g = (color >> 8) & 0xFF;
  earColor.r = (color >> 16) & 0xFF;
}
ros::Subscriber<std_msgs::UInt32> earColorSubscriber("/head/ear_color", &ear_color_callback);


////////////////////////////////////////////////////////////////////////////////
void setup() {
  // This is for Trinket 5V 16MHz, you can remove these three lines if you are not using a Trinket
#if defined (__AVR_ATtiny85__)
  if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
#endif
  // End of trinket special code

  pinMode(LED_BUILTIN, OUTPUT); // heartbeat LED
  eyesStrip.begin();
  eyesStrip.show(); // Initialize all pixels to 'off'
  earsStrip.begin();
  earsStrip.show(); // Initialize all pixels to 'off'
  randomSeed(analogRead(0));

// NOTE! Eyes are GRB + W!
  // eyeColor = {0, 0, DEFAULT_EYE_BRIGHTNESS, 0}; // default to blue
  // eyeColor = {0,  DEFAULT_EYE_BRIGHTNESS, 0}; // default to green
  // eyeColor = {0,  DEFAULT_EYE_BRIGHTNESS, DEFAULT_EYE_BRIGHTNESS, 0}; // aqua
  eyeColor = {DEFAULT_EYE_BRIGHTNESS, DEFAULT_EYE_BRIGHTNESS, DEFAULT_EYE_BRIGHTNESS, DEFAULT_EYE_BRIGHTNESS}; // default to mixed white

  earColor = {DEFAULT_EAR_BRIGHTNESS, DEFAULT_EAR_BRIGHTNESS, DEFAULT_EAR_BRIGHTNESS}; // R,G,B - default to white
  //earColor = {0, DEFAULT_EAR_BRIGHTNESS, 0}; // G,R,B 

  // Initialize ROS
  nh.initNode();
  nh.loginfo("ROS Init Start");
  nh.subscribe(eyeCommandSubscriber);
  nh.subscribe(eyeColorSubscriber);
  nh.subscribe(earCommandSubscriber);
  nh.subscribe(earColorSubscriber);

  // blink LED on the board at startup
  for (int i = 0; i < 10; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }


#ifdef FEATHER_M4_EXPRESS
  onboard_neopixel.begin();
  onboard_neopixel.show(); // Initialize all pixels to 'off'

  // Startup Dance
  for (int i = 0; i < 5; i++) {
    onboard_neopixel.setPixelColor(0, 127, 0, 0);
    onboard_neopixel.show();
    delay(100);  
    onboard_neopixel.setPixelColor(0, 0, 127, 0);
    onboard_neopixel.show();
    delay(100);  
    onboard_neopixel.setPixelColor(0, 0, 0, 127);
    onboard_neopixel.show();
    delay(100);  
  }
  onboard_neopixel.setPixelColor(0, 16, 16, 16);
  onboard_neopixel.show();
#endif

  
  //EyeCmdState = EYES_OFF;  // Off by default, until ROS turns them on
  EyeCmdState = EYES_AUTO_BLINK;  // For Testing.  DAVES TODO: comment this line out
 
  // EyesOn(eyesStrip.Color(eyeColor.r, eyeColor.g, eyeColor.b, eyeColor.w) ); // DAVES TODO: comment this line out

  EarColorWipe(earsStrip.Color(  250,   50,    50), 10); // RGB white
  earsStrip.show();            // 

  
  nh.loginfo("Head Arduino started");
}


////////////////////////////////////////////////////////////////////////////////
void loop() {

  // nh.loginfo("Head Arduino: LOOP"); 

  
  if(EYES_AUTO_BLINK == EyeCmdState) {
    EyesBlink(eyesStrip.Color(eyeColor.r, eyeColor.g, eyeColor.b, eyeColor.w), 5); // Color, delay
  }
  
  // Random delays between blinks, between 1 and 6 seconds
  int randTime = random(1, 20); // DAVES 60
  HeartBeatLedState = false;
  FadeOnState = false;
  
  for(int i=0; i<randTime; i++) {

    // this loop executes once every second
    digitalWrite(LED_BUILTIN, HeartBeatLedState); // on every 2 seconds
    HeartBeatLedState = !HeartBeatLedState;

  // Onboard Neopixel Heartbeat
#ifdef FEATHER_M4_EXPRESS
  if(onboardNeoPixelColor++ > 2) {
    onboardNeoPixelColor = 0;
  }
  onboard_neopixel.setPixelColor(0, (onboardNeoPixelIntensity << (onboardNeoPixelColor*8) ));
  onboard_neopixel.show();
#endif  

   
    // nh.loginfo("Head Arduino:    Inner Loop"); 

    // Do Ear effect. Must take 1 second, including Ros spinonce commands (every 100 ms or so)  
    updateEar();
    
    // ROS Heartbeat / Communicate with ROS
    nh.spinOnce();
    //delay(100); // delay as needed to make up 100ms

    if(blinkNow) {
      blinkNow = false; // reset flag
      break; // exit wait loop
    }
  }
}


////////////////////////////////////////////////////////////////////////////////
void updateEar() { // This function needs to take close to 1 second

  if(0 == EarCmdMode) {
    // Ear Lights off
    earsFastColorFill(BLACK);
    for(int i=0; i<10; i++) {
      nh.spinOnce();
      delay(99); // delay as needed to make up 1 second
    }
  }
  else if(1 == EarCmdMode) {
    // Slow fade on/off.  Adjust max_brightness and step_delay for desired effect
    uint8_t max_brightness = 64; 
    uint8_t step_delay = 40;
    // color, min_brightness, max_brightness,  wait
    if(FadeOnState) {
      RampDown(earColor.r, earColor.g, earColor.b, 20, max_brightness, step_delay); // fade to off
    }
    else {
      RampUp(earColor.r, earColor.g, earColor.b, 20, max_brightness, step_delay);  // fade to on
    }
    FadeOnState = !FadeOnState;
    
    // earsFastColorFill(BLACK); // TODO DEBUG

  }
  else if(2 == EarCmdMode) {
    // Spin Pattern
    SpinLight(earColor.r, earColor.g, earColor.b, 50);
  }
  else {  // unknown mode!  add error check here?
    for(int i=0; i<10; i++) {
      nh.spinOnce();
      delay(99); // delay as needed to make up 1 second
    }
  }
}



/*** EXAMPLES
//  colorWipe(earsStrip.Color(WHITE_LEVEL, WHITE_LEVEL, WHITE_LEVEL), 5); // White
  //delay(1000);

  digitalWrite(LED_BUILTIN, LOW);
  colorWipe(earsStrip.Color(RED_LEVEL, 0, 0), 5); // Red
  delay(1000);
  
  digitalWrite(LED_BUILTIN, HIGH);
  colorWipe(earsStrip.Color(0, GREEN_LEVEL, 0), 5); // Green
  delay(1000);
  
  digitalWrite(LED_BUILTIN, LOW);
  colorWipe(earsStrip.Color(0, 0, BLUE_LEVEL), 5); // Blue
  delay(1000);

  uint8_t effect_speed_delay = 100;
  //colorWipe(earsStrip.Color(0, 0, 0, 255), 50); // White RGBW
  // Send a theater pixel chase in...
  //theaterChase(earsStrip.Color(127, 127, 127), effect_speed_delay); // White
  //theaterChase(earsStrip.Color(127, 0, 0), effect_speed_delay); // Red
  // theaterChase(earsStrip.Color(0, 0, 127), effect_speed_delay); // Blue

    //earsFastColorFill(0,0,0);
 //   delay(500);

  //  earsStrip.setPixelColor(PIXELS_PER_RING, r, g, b);
  
    earsFastColorFill(127,0,0);
    delay(500);
  
    earsFastColorFill(0,127,0);
    delay(500);
  
    earsFastColorFill(0,0,127);
    delay(500);

    earsFastColorFill(0,0,127);
    delay(500);

  for(int i=0; i<16; i++) {
    Pulse(0, 0, 120, 100);
  }

  for(int i=0; i<10; i++) {
    SpinLight(200,200,200, 50);
  }

  RampUp(255,255,255, 0, 50, 10); // R,G,B, min brightness, max brightness, delay
  for(int i=0; i<10; i++) {
    Fade(127,127,127, 50, 127, 10); // R,G,B, min brightness, max brightness, delay
    //Fade(255,255,255, 50, 255, 10); // R,G,B, min brightness, max brightness, delay
  } 

   
//  rainbow(10);
  rainbowCycle(10);
  //theaterChaseRainbow(effect_speed_delay);

  delay(100); // delay as needed to make up 100ms
***/


////////////////////////////////////////////////////////////////////////////////
// Subroutines

// EYES ON
// Quickly fill all the dots with a color.  Is there a better function for this?
void EyesOn(uint32_t c) {
  for (uint16_t i = 0; i < eyesStrip.numPixels(); i++) {
    eyesStrip.setPixelColor(i, c);
  }
  eyesStrip.show(); // move to end of loop?
}


// EYES OFF
// Quickly fill all the dots with a color.  Is there a better function for this?
void EyesOff() {
  for (uint16_t i = 0; i < eyesStrip.numPixels(); i++) {
    eyesStrip.setPixelColor(i, eyesStrip.Color(0, 0, 0));
  }
  eyesStrip.show(); // move to end of loop?
}

// WAKE UP
void WakeUp(uint8_t wait) {

  for (uint16_t i = 0; i < 255; i++) {
    uint16_t LeftEyeRightSide = i;
    uint16_t LeftEyeLeftSide = (PIXELS_PER_RING - 1) - i;
    uint16_t RightEyeRightSide = PIXELS_PER_RING + i;
    uint16_t RightEyeLeftSide = ((PIXELS_PER_RING * 2) - 1) - i;

    eyesStrip.setPixelColor(LeftEyeRightSide, BLACK);
    eyesStrip.setPixelColor(LeftEyeLeftSide, BLACK );
    eyesStrip.setPixelColor(RightEyeRightSide, BLACK);
    eyesStrip.setPixelColor(RightEyeLeftSide, BLACK );
    eyesStrip.show();
    delay(wait);
  }
}

////////////////////////////////////////////////////////////
// BLINK EYES!
// Assumes eyes are on.  Blink Off then back On
// 24 LEDs per ring, 12 each side, 2 rings
const int LeftRingStart = 0;
const int RightRingStart = PIXELS_PER_RING;
const int HALF_RING = PIXELS_PER_RING / 2;

void EyesBlink(uint32_t c, uint8_t wait) {

  // Turn LEDs Off
  // for(int16_t i=(HALF_RING-1); i>=0; i--) {
  for (uint16_t i = 0; i < HALF_RING; i++) {
    uint16_t LeftEyeRightSide = i;
    uint16_t LeftEyeLeftSide = (PIXELS_PER_RING - 1) - i;
    uint16_t RightEyeRightSide = PIXELS_PER_RING + i;
    uint16_t RightEyeLeftSide = ((PIXELS_PER_RING * 2) - 1) - i;

    eyesStrip.setPixelColor(LeftEyeRightSide, BLACK);
    eyesStrip.setPixelColor(LeftEyeLeftSide, BLACK );
    eyesStrip.setPixelColor(RightEyeRightSide, BLACK);
    eyesStrip.setPixelColor(RightEyeLeftSide, BLACK );
    eyesStrip.show();
    delay(wait);
  }
  delay(100);

  // Turn LEDs Back ON
  for (int16_t i = (HALF_RING - 1); i >= 0; i--) {
    //  for(uint16_t i=0; i<HALF_RING; i++) {
    uint16_t LeftEyeRightSide = i;
    uint16_t LeftEyeLeftSide = (PIXELS_PER_RING - 1) - i;
    uint16_t RightEyeRightSide = PIXELS_PER_RING + i;
    uint16_t RightEyeLeftSide = ((PIXELS_PER_RING * 2) - 1) - i;

    eyesStrip.setPixelColor(LeftEyeRightSide, c);
    eyesStrip.setPixelColor(LeftEyeLeftSide, c );
    eyesStrip.setPixelColor(RightEyeRightSide, c);
    eyesStrip.setPixelColor(RightEyeLeftSide, c );
    eyesStrip.show();
    delay(wait);
  }
}

// NEOPIXEL EFFECTS ROUTINES
////////////////////////////////////////////////////////////

void earsFastColorFill(uint8_t r, uint8_t g, uint8_t b) { // RGB version

  for (uint16_t i = 0; i < (PIXELS_PER_RING*2); i++) {
    earsStrip.setPixelColor(i, earsStrip.Color(r, g, b));
  }
  earsStrip.show(); 
}

void earsFastColorFill(uint32_t c) { // Color version

  for (uint16_t i = 0; i < (PIXELS_PER_RING*2); i++) {
    earsStrip.setPixelColor(i, c);
  }
  earsStrip.show(); 
}



// Fill the dots one after the other with a color
void EarColorWipe(uint32_t c, uint8_t wait) {
  for (uint16_t i = 0; i < earsStrip.numPixels(); i++) {
    earsStrip.setPixelColor(i, c);
    earsStrip.show();
    delay(wait);
  }
}


// PULSE - TODO DO THIS FOR BOTH EARS AT ONCE
void Pulse(uint8_t r, uint8_t g, uint8_t b, uint8_t wait) {
  uint8_t intensity = 0;
  const uint8_t BRIGHTNESS_STEP = 5;
  for (uint16_t i = 0; i < (PIXELS_PER_RING - 4); i++) {
    for (uint16_t pos = 0; pos < 5; pos++) {
      intensity = (pos) * BRIGHTNESS_STEP;
      earsStrip.setPixelColor(i + pos, r * intensity, g * intensity, b * intensity);
    }
    earsStrip.show();
    nh.spinOnce();
    delay(wait);
  }
}



// SPINLIGHT - Spin a single light arond the circle for both ears (right clockwise, left counter-clockwise)
void SpinLight(uint8_t r, uint8_t g, uint8_t b, uint8_t wait) {
  earsStrip.setPixelColor(PIXELS_PER_RING-1, r, g, b);
  earsStrip.setPixelColor(PIXELS_PER_RING, r, g, b);
  earsStrip.show();
  nh.spinOnce();
  delay(wait);
  for (uint16_t i = 1; i < (PIXELS_PER_RING); i++) {
    earsStrip.setPixelColor(PIXELS_PER_RING -(i+1), r, g, b); // right ear
    earsStrip.setPixelColor((i+PIXELS_PER_RING), r, g, b);  // left ear
    earsStrip.setPixelColor(PIXELS_PER_RING - i, 0,0,0);
    earsStrip.setPixelColor((i+PIXELS_PER_RING)-1, 0,0,0);
    earsStrip.show();
    nh.spinOnce();
    delay(wait);
  }
  /* IS THIS NEEDED?
  earsStrip.setPixelColor(0, 0, 0, 0);
  earsStrip.setPixelColor((PIXELS_PER_RING*2)-1, 0, 0, 0);
  earsStrip.show();
  */

}

// RAMP UP - ramp all leds together
// all values are 0 - 255
void RampUp(uint8_t r, uint8_t g, uint8_t b, uint8_t min_brightness, uint8_t max_brightness, uint8_t wait) { 
  int32_t intensity = 0;
  const int32_t BRIGHTNESS_STEP = 2;

  for (int32_t step = (min_brightness/BRIGHTNESS_STEP); step < max_brightness/BRIGHTNESS_STEP; step++) {
    intensity = step * BRIGHTNESS_STEP;
    earsFastColorFill( ((r * intensity) / 255), ((g * intensity) / 255), ((b * intensity) / 255) );
    earsStrip.show();
    delay(wait);
  }
}

// RAMP DOWN - ramp down all leds together
// all values are 0 - 255
void RampDown(uint8_t r, uint8_t g, uint8_t b, uint8_t min_brightness, uint8_t max_brightness, uint8_t wait) { 
  int32_t intensity = 0;
  const int32_t BRIGHTNESS_STEP = 2;

  for (int32_t step = max_brightness/BRIGHTNESS_STEP; step >= (min_brightness/BRIGHTNESS_STEP); step--) {
    intensity = step * BRIGHTNESS_STEP;
    earsFastColorFill( ((r * intensity) / 255), ((g * intensity) / 255), ((b * intensity) / 255) );
    earsStrip.show();
    delay(wait);
  }
}


// FADE - fade all leds together
void Fade(uint8_t r, uint8_t g, uint8_t b, uint8_t min_brightness, uint8_t max_brightness, uint8_t wait) { // 125 = medium
  int32_t intensity = 0;
  const int32_t BRIGHTNESS_STEP = 2;
  //max_brightness = 255;
  //min_brightness = 50;

  // ramp up
  for (int32_t step = (min_brightness/BRIGHTNESS_STEP); step < max_brightness/BRIGHTNESS_STEP; step++) {
    intensity = step * BRIGHTNESS_STEP;
    earsFastColorFill( ((r * intensity) / 255), ((g * intensity) / 255), ((b * intensity) / 255) );
    earsStrip.show();
    nh.spinOnce();
    delay(wait);
  }

  // ramp down
  for (int32_t step = max_brightness/BRIGHTNESS_STEP; step >= (min_brightness/BRIGHTNESS_STEP); step--) {
    intensity = step * BRIGHTNESS_STEP;
    earsFastColorFill( ((r * intensity) / 255), ((g * intensity) / 255), ((b * intensity) / 255) );
    earsStrip.show();
    nh.spinOnce();
    delay(wait);
  }
    delay(wait * 10);

}


void rainbow(uint8_t wait) {
  uint16_t i, j;

  for (j = 0; j < 256; j++) {
    for (i = 0; i < earsStrip.numPixels(); i++) {
      earsStrip.setPixelColor(i, Wheel((i + j) & 255));
    }
    earsStrip.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for (j = 0; j < 256 * 5; j++) { // 5 cycles of all colors on wheel
    for (i = 0; i < earsStrip.numPixels(); i++) {
      earsStrip.setPixelColor(i, Wheel(((i * 256 / earsStrip.numPixels()) + j) & 255));
    }
    earsStrip.show();
    delay(wait);
  }
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait) {
  for (int j = 0; j < 10; j++) { //do 10 cycles of chasing
    for (int q = 0; q < 3; q++) {
      for (uint16_t i = 0; i < earsStrip.numPixels(); i = i + 3) {
        earsStrip.setPixelColor(i + q, c);  //turn every third pixel on
      }
      earsStrip.show();

      delay(wait);

      for (uint16_t i = 0; i < earsStrip.numPixels(); i = i + 3) {
        earsStrip.setPixelColor(i + q, 0);      //turn every third pixel off
      }
    }
  }
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
  for (int j = 0; j < 256; j++) {   // cycle all 256 colors in the wheel
    for (int q = 0; q < 3; q++) {
      for (uint16_t i = 0; i < earsStrip.numPixels(); i = i + 3) {
        earsStrip.setPixelColor(i + q, Wheel( (i + j) % 255)); //turn every third pixel on
      }
      earsStrip.show();

      delay(wait);

      for (uint16_t i = 0; i < earsStrip.numPixels(); i = i + 3) {
        earsStrip.setPixelColor(i + q, 0);      //turn every third pixel off
      }
    }
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return earsStrip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return earsStrip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return earsStrip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
