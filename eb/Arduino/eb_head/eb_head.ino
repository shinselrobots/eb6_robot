// EB Head controls Eye and Ear lights, and maybe sensors in the future

// Adafruit NeoPixel used for the eyes
// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

// Optional DMA for Neopixels (to prevent conflict with serial port)
#include <Adafruit_NeoPixel_ZeroDMA.h>
#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix_ZeroDMA.h>


// If not using a Feather, you might need to comment out one or both of these lines:
#define FEATHER_M4_EXPRESS     // if using the M4 Express board
#define USE_USBCON  // NEEDED FOR CPU with Built-in USB.  ATmega32u4 - Feather 32u4, Feather M4, LEONARDO...


// ROS
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
//#include <std_msgs/Empty.h>
//#include <geometry_msgs/Twist.h>
//#include <sensor_msgs/Joy.h>
//#include <behavior_common/CommandState.h>
#include <ros/time.h>
//#include "RobotConstants.h"

// Structures
typedef struct { uint8_t g; uint8_t r; uint8_t b; uint8_t w; } EyeColor_t;
typedef struct { uint8_t g; uint8_t r; uint8_t b; } EarColor_t;

enum EYE_CMD_STATE_T {
  EYES_OFF = 0,
  EYES_ON_SOLID,
  EYES_AUTO_BLINK,
};


/////////////////////////////////////////////////////////////////////////////////////
// Constants

#define NeoEyesPIN A2     // Supports DMA
#define NeoEarsPIN A4     // Supports DMA
#define Matrix1PIN 12     // Supports DMA
#define Matrix2PIN MOSI   // Supports DMA

#define TouchSensorPIN A5

#define PIXELS_PER_RING 16
#define MATRIX_ROWS 4
#define MATRIX_COLUMNS 8

#define DEFAULT_EYE_BRIGHTNESS   20 // med-low brightness
#define DEFAULT_EAR_BRIGHTNESS   50 // med-low brightness

const uint32_t BLACK = 0;


/////////////////////////////////////////////////////////////////////////////////////
// Global Variables

// DEBUG NOTE: Make sure 12v servo power is on, or the lights won't work!
bool            NeoPixels_Debug_On = false; // DEBUG: set true for running NeoPixels on startup
EYE_CMD_STATE_T EyeCmdState = EYES_OFF; 
uint8_t         EarCmdMode = 0;
bool            HeartBeatLedState = false;
bool            FadeOnState = false;
uint32_t        onboardNeoPixelColor = 0;
const uint8_t   onboardNeoPixelIntensity = 32; // 0 - 255
bool            interrupt_loop = false; // interrupt to break from the ear update loop when new commands received
bool            touch_detected = false;
ros::NodeHandle nh;

// NOTE! Eyes are RGB + White!
EyeColor_t  eyeColor = {0, 0, 0x2f, 0}; // default to blue
// EyeColor_t  eyeColor = {DEFAULT_EYE_BRIGHTNESS, DEFAULT_EYE_BRIGHTNESS, DEFAULT_EYE_BRIGHTNESS, DEFAULT_EYE_BRIGHTNESS}; // default to mixed white

EarColor_t  earColor = {0, 0, 0x2f}; // default to blue
// EarColor_t  earColor = {DEFAULT_EAR_BRIGHTNESS, DEFAULT_EAR_BRIGHTNESS, DEFAULT_EAR_BRIGHTNESS}; // R,G,B - default to white

// Use DMA NeoPixel Library
Adafruit_NeoPixel_ZeroDMA eyesStrip = Adafruit_NeoPixel_ZeroDMA((PIXELS_PER_RING * 2), NeoEyesPIN, NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel_ZeroDMA earsStrip = Adafruit_NeoPixel_ZeroDMA((PIXELS_PER_RING * 2), NeoEarsPIN, NEO_GRB + NEO_KHZ800);

Adafruit_NeoMatrix_ZeroDMA matrix1(MATRIX_ROWS, MATRIX_COLUMNS, Matrix1PIN,
  NEO_MATRIX_COLUMNS + NEO_MATRIX_PROGRESSIVE,
  NEO_GRB);

Adafruit_NeoMatrix_ZeroDMA matrix2(MATRIX_ROWS, MATRIX_COLUMNS, Matrix2PIN,
  NEO_MATRIX_TOP     + NEO_MATRIX_RIGHT +
  NEO_MATRIX_COLUMNS + NEO_MATRIX_PROGRESSIVE,
  NEO_GRB);

const uint16_t colors[] = {
  matrix1.Color(255, 0, 0), matrix1.Color(0, 255, 0), matrix1.Color(0, 0, 255), matrix1.Color(0, 0, 0) };


// Use Standard NeoPixel library
//Adafruit_NeoPixel eyesStrip = Adafruit_NeoPixel((PIXELS_PER_RING * 2), NeoEyesPIN, NEO_GRBW + NEO_KHZ800);
//Adafruit_NeoPixel earsStrip = Adafruit_NeoPixel((PIXELS_PER_RING * 2), NeoEarsPIN, NEO_GRB + NEO_KHZ800);

#ifdef FEATHER_M4_EXPRESS
// enable the single onboard Neopixel
#define NUMBER_OF_ONBOARD_NEOPIXELS 1
#define ONBOARD_NEOPIXEL_PIN 8  
Adafruit_NeoPixel onboard_neopixel = Adafruit_NeoPixel(NUMBER_OF_ONBOARD_NEOPIXELS, ONBOARD_NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
#endif


////////////////////////////////////////////////////////////////////////////////
// ROS node subscribers and publishers

// EYE MODE COMMAND: To test, try this: rostopic pub -1 /head/eye_cmd std_msgs/UInt16  2 // Eyes Blink
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
  interrupt_loop = true; // force wait loop to exit immediately

}
ros::Subscriber<std_msgs::UInt16> eyeCommandSubscriber("/head/eye_cmd", &eye_cmd_callback);

// EYE COLOR COMMAND: To test, try this: rostopic pub -1 /head/eye_color std_msgs/UInt32 0x002f2f (green/blue)
void eye_color_callback(const std_msgs::UInt32& cmd_msg) {
  uint32_t color = cmd_msg.data;
  nh.loginfo("Head Arduino Setting EYE COLOR");
  eyeColor.b = color & 0xFF;
  eyeColor.g = (color >> 8) & 0xFF;
  eyeColor.r = (color >> 16) & 0xFF;
  eyeColor.w = (color >> 24) & 0xFF;
  interrupt_loop = true; // force wait loop to exit immediately
}
ros::Subscriber<std_msgs::UInt32> eyeColorSubscriber("/head/eye_color", &eye_color_callback);


// EAR MODE COMMAND: Set the ears light pattern. To test, try this: rostopic pub -1 /head/ear_cmd std_msgs/UInt16  3 // rainbow
void ear_cmd_callback(const std_msgs::UInt16& cmd_msg) {
  EarCmdMode = cmd_msg.data;

  // NOTE: Actual Mode selection is handled later
  if(0 == EarCmdMode) {
    nh.loginfo("Head Arduino: EAR COMMAND = OFF");
  }
  //else {
  //  nh.loginfo("Head Arduino: EAR COMMAND Received");
  //}
  interrupt_loop = true; // force wait loop to exit immediately
}
ros::Subscriber<std_msgs::UInt16> earCommandSubscriber("/head/ear_cmd", &ear_cmd_callback);


// EAR COLOR: To test, try this: rostopic pub -1 /head/ear_color std_msgs/UInt32 "0x002f00" (green)
void ear_color_callback(const std_msgs::UInt32& cmd_msg) {
  uint32_t color = cmd_msg.data;

  nh.loginfo("Head Arduino Got EAR COLOR message");
  earColor.b = color & 0xFF;
  earColor.g = (color >> 8) & 0xFF;
  earColor.r = (color >> 16) & 0xFF;
  interrupt_loop = true; // force wait loop to exit immediately

}
ros::Subscriber<std_msgs::UInt32> earColorSubscriber("/head/ear_color", &ear_color_callback);

std_msgs::Int16 touchMsg;  // touch sensor on top of head
ros::Publisher pub_touch("/head_touch", &touchMsg);

void check_touch_sensor() {
  if(!touch_detected) {
    int touchSensorValue = digitalRead(TouchSensorPIN);
    if(touchSensorValue > 0) {
      touch_detected = true;
      touchMsg.data = touchSensorValue;
      pub_touch.publish(&touchMsg);

    }
  }
}

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
  // earsStrip.setBrightness(50);
  earsStrip.show(); // Initialize all pixels to 'off'
  randomSeed(analogRead(0));

  pinMode(TouchSensorPIN, INPUT); 
 
  //EyesOff();

// NOTE! Eyes are GRB + W!
  // eyeColor = {0, 0, DEFAULT_EYE_BRIGHTNESS, 0}; // default to blue
  // eyeColor = {0,  DEFAULT_EYE_BRIGHTNESS, 0}; // default to green
  // eyeColor = {0,  DEFAULT_EYE_BRIGHTNESS, DEFAULT_EYE_BRIGHTNESS, 0}; // aqua
  //earColor = {0, DEFAULT_EAR_BRIGHTNESS, 0}; // G,R,B 

  // Initialize ROS
  nh.initNode();
  nh.loginfo("ROS Init Start");
  nh.advertise(pub_touch);
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
  for (int i = 0; i < 3; i++) {
    onboard_neopixel.setPixelColor(0, 127, 0, 0); // pixel index, color
    onboard_neopixel.show();
    delay(100);  
    onboard_neopixel.setPixelColor(0, 0, 127, 0);
    onboard_neopixel.show();
    delay(100);  
    onboard_neopixel.setPixelColor(0, 0, 0, 127);
    onboard_neopixel.show();
    delay(100);  
  }
  onboard_neopixel.setPixelColor(0, 127, 0, 0); // Red, meaning not running yet (will cycle when running)
  onboard_neopixel.show();
  delay(100);  
#endif

  // Set NeoPixels On or off at startup
  if (NeoPixels_Debug_On) {
    // DEBUG NOTE: Make sure 12v servo power is on, or the lights won't work!
    EyeCmdState = EYES_AUTO_BLINK;
    EarCmdMode = 3;
  }
  else {
    EyeCmdState = EYES_OFF;
    EarCmdMode = 0;
  }

  matrix1.begin();
  matrix2.begin();
  matrix1.setTextWrap(false);
  matrix2.setTextWrap(false);
  matrix1.setTextColor(colors[0]);
  matrix2.setTextColor(colors[0]);
  matrix1.setBrightness(8);
  matrix2.setBrightness(8);
  matrix1.clear();
  matrix2.clear();
  matrix1.show();
  matrix2.show();


  // For DEBUG: Show NeoPixel Ear leds are working
  //doubleTheaterChase(16, earsStrip.Color(255, 255, 255), 100);
  //doubleRainbowCycle(16, 5);

  
  
  nh.loginfo("Head Arduino: started");




  // DEBUG: For Installation, Light top two pixels of each eye
  /*
  eyesStrip.setPixelColor( 0, eyesStrip.Color(0, 55, 0, 0)); // Green = Starting LED
  eyesStrip.setPixelColor(15, eyesStrip.Color(55, 0, 0, 0)); // Red = Ending LED
  eyesStrip.setPixelColor(16, eyesStrip.Color(0, 55, 0, 0));
  eyesStrip.setPixelColor(31, eyesStrip.Color(55, 0, 0, 0));
  eyesStrip.show();            // 
  // Note: delay not needed. Leds will stay lit until ROS command received.
  //delay(5000);

  */

  // DEBUG: For Installation, Light back pixel of each ear
  /*
  earsStrip.setPixelColor( 0, earsStrip.Color(00, 00, 55));
  earsStrip.setPixelColor(16, earsStrip.Color(00, 00, 55));
  earsStrip.show();            // 
  delay(5000);
  */

  //earsFastColorFill(BLACK); // TODO use new function for this (see webpage)
}


////////////////////////////////////////////////////////////////////////////////
void loop() {

  // nh.loginfo("Head Arduino: LOOP"); 

  // For debug, display patterns on ears
  // doubleRainbowCycle(PIXELS_PER_RING, 5);
  // doubleTheaterChase(PIXELS_PER_RING, earsStrip.Color(255, 255, 255), 100);
    
  if(EYES_AUTO_BLINK == EyeCmdState) {
    EyesBlink(eyesStrip.Color(eyeColor.r, eyeColor.g, eyeColor.b, eyeColor.w), 5); // Color, delay
  }
  
  // Do other stuf for a random time to delay between blinks
  int randTime = random(1, 5); // DAVES 60
  HeartBeatLedState = false;
  FadeOnState = false;
  
  for(int i=0; i<randTime; i++) {

    // each time around the loop should take one second. After done (1 sec * randTime), the eyes blink
    interrupt_loop = false;
    touch_detected = false;

    digitalWrite(LED_BUILTIN, HeartBeatLedState); // on every 2 seconds
    HeartBeatLedState = !HeartBeatLedState;


  // Onboard Neopixel Heartbeat
#ifdef FEATHER_M4_EXPRESS
    onboard_neopixel.setPixelColor(0, (onboardNeoPixelIntensity << (onboardNeoPixelColor*8) ));
    onboard_neopixel.show();
    if(onboardNeoPixelColor++ > 2) {
      onboardNeoPixelColor = 0;
    }
#endif  

   
    // nh.loginfo("Head Arduino:    Inner Loop"); 

    // Do Ear effect. Must take 1 second, including Ros spinonce commands (every 100 ms or so)  
    updateEar();

    if(interrupt_loop) {
      break;      
    }
    
    nh.spinOnce(); // ROS Heartbeat / Communicate with ROS
    //delay(100); // delay as needed to make up 100ms

    if(interrupt_loop) {
      break; // exit wait loop
    }
  } // 1 second loop

 
}




////////////////////////////////////////////////////////////////////////////////
void updateEar() { // This function needs to take close to 1 second

  if(0 == EarCmdMode) {
    // Ear Lights off
    earsFastColorFill(BLACK);
    matrix1.clear();
    matrix2.clear();
    matrix1.show();
    matrix2.show();

    for(int i=0; i<10; i++) {
      nh.spinOnce();
      if(interrupt_loop) {
        break;      
      }
      check_touch_sensor();
      delay(99); // delay as needed to make up 1 second
    }
  }
  else if(1 == EarCmdMode) {
    // Slow fade on/off.  Adjust max_brightness and step_delay for desired effect
    uint8_t max_brightness = 127; // 64; 
    uint8_t step_delay = 20;
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
    // Spin Pattern in white
    doubleTheaterChase(16, earsStrip.Color(255, 255, 255), 50); // Bright White
    // doubleTheaterChase(16, earsStrip.Color(earColor.r, earColor.g, earColor.b), 150); // whatever current color is
    // doubleTheaterChase(16, earsStrip.Color(127, 127, 127), 50); // White
    // SpinLight(earColor.r, earColor.g, earColor.b, 250); // delay = 50
  }

  else if(3 == EarCmdMode) {
    // Rainbow Pattern
    doubleRainbowCycle(PIXELS_PER_RING, 5);
    // SpinLight(earColor.r, earColor.g, earColor.b, 250); // delay = 50
  }

  else if(4 == EarCmdMode) {
    // Spin Pattern in blue
    doubleTheaterChase(16, earsStrip.Color(0, 0, 255), 50); // Blue
    // doubleTheaterChase(16, earsStrip.Color(earColor.r, earColor.g, earColor.b), 150); // whatever current color is
    // doubleTheaterChase(16, earsStrip.Color(127, 127, 127), 50); // White
    // SpinLight(earColor.r, earColor.g, earColor.b, 250); // delay = 50
  }

  else {  // unknown mode!  add error check here?
    for(int i=0; i<10; i++) {
      nh.spinOnce();
      delay(99); // delay as needed to make up 1 second
    }
  }
}


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



////////////////////////////////////////////////////////////
// BLINK EYES!
// Assumes eyes are on.  Blink Off then back On
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

  check_touch_sensor();
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


void updateMatrix(uint8_t wait) {
  // wait is how long the calling routine waits between loops
  uint16_t global_color = uint16_t(0xFFFFFF);
  //delay_min = 50 / wait;
  //delay_max = 300 / wait;
  int rand_delay = random(0,100);
  if(rand_delay > 90) {
    for(int x=0; x < MATRIX_COLUMNS; x++) {
      for(int y=0; y < MATRIX_ROWS; y++) {
        int16_t randColor = 0; // default to off
        int randOn = random(0, 10);
        if(randOn < 3) {
          randColor = global_color;
        }
        matrix1.drawPixel(y,x, randColor);
        matrix2.drawPixel(y,x, randColor);
      }
    }
    matrix1.show();
    matrix2.show();
  }
}


////////////////////////////////////////////////////////////////////////////////
// NEOPIXEL EFFECTS ROUTINES

// synchronized rainbow cycle for both ears
void doubleRainbowCycle(uint32_t pixelsPerRing, uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*1; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< pixelsPerRing; i++) {
      earsStrip.setPixelColor(i, Wheel(((i * 256 / pixelsPerRing) + j) & 255));
      
      earsStrip.setPixelColor( ((pixelsPerRing*2) - i), Wheel(((i * 256 / pixelsPerRing) + j) & 255));
    }
    earsStrip.show();
    updateMatrix(wait);
    delay(wait);
    check_touch_sensor();
    nh.spinOnce();

  }
}



//Theatre-style crawling lights for both ears
void doubleTheaterChase(uint32_t pixelsPerRing, uint32_t c, uint8_t wait) {
  for (int j=0; j<10; j++) {  //do 10 cycles of chasing

    for (int q=0; q < 3; q++) {
      for (uint16_t i=0; i < pixelsPerRing; i=i+3) {
        earsStrip.setPixelColor( pixelsPerRing - (i+q), c);    //turn every third pixel on

        earsStrip.setPixelColor( pixelsPerRing + (i+q), c);    //turn every third pixel on
        
      }
      earsStrip.show();

      for(int mydelay=0; mydelay<10; mydelay++) {
        updateMatrix(wait/10);
        delay(wait/10);
        check_touch_sensor();
        nh.spinOnce();
      }

      for (uint16_t i=0; i < pixelsPerRing; i=i+3) {
        earsStrip.setPixelColor( pixelsPerRing - (i+q), 0);        //turn every third pixel off

        earsStrip.setPixelColor( pixelsPerRing + (i+q), 0);        //turn every third pixel off
      
      
      }
    }
  }
}


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
    check_touch_sensor();
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
    check_touch_sensor();
    nh.spinOnce();
    delay(wait);
    if(interrupt_loop) {
      break;      
    }
  }
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
    check_touch_sensor();
    delay(wait);
    if(interrupt_loop) {
      break;      
    }
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
    check_touch_sensor();
    delay(wait);
    if(interrupt_loop) {
      break;      
    }
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
    check_touch_sensor();
    nh.spinOnce();
    delay(wait);
    if(interrupt_loop) {
      break;      
    }
  }

  // ramp down
  for (int32_t step = max_brightness/BRIGHTNESS_STEP; step >= (min_brightness/BRIGHTNESS_STEP); step--) {
    intensity = step * BRIGHTNESS_STEP;
    earsFastColorFill( ((r * intensity) / 255), ((g * intensity) / 255), ((b * intensity) / 255) );
    earsStrip.show();
    check_touch_sensor();
    nh.spinOnce();
    if(interrupt_loop) {
      break;      
    }
    delay(wait);
  }
  for (int32_t end_delay = 0; end_delay < 10; end_delay++) {
    check_touch_sensor();
    delay(wait);
    if(interrupt_loop) {
      break;      
    }
  } 

}


// Wheel! Input a value 0 to 255 to get a color value.
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
