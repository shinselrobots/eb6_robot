/* EB Body Feather M4.  Currently handles:
   - Bluetooth Phone connection
   - IMU: Sparkfun BNO086
   - Battery Voltage via A2D pin
   - Sharp IR distance sensors via A2D
   - Ultrasonic sensor via A2D
   - NEOPIXEL LED Strip: Command is: First byte: LED address, remaining 3 bytes: RGB color
*/

// If not using a Feather, you might need to comment out one or both of these lines:
#define FEATHER_M4_EXPRESS  // if using the M4 Express board
#define USE_USBCON          // NEEDED FOR CPU with Built-in USB.  ATmega32u4 - Feather 32u4, Feather M4, LEONARDO...

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

// Optional DMA for Neopixels (to prevent conflict with serial port)
#include <Adafruit_NeoPixel_ZeroDMA.h>

// ROS
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <geometry_msgs/Point32.h>
#include <ros/time.h>
#include <sensor_msgs/Joy.h>
#include <behavior_msgs/CommandState.h>
#include <system_status_msgs/SystemStatus.h>


// For Sparkfun BNO086 IMU:
#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h"  // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BNO08x
BNO08x myIMU;
// Per Sparkfun recommendation and example code, IMU connected with Interrupt and Reset as follows:
#define BNO08X_INT 5
#define BNO08X_RST 6
#define BNO08X_ADDR 0x4B  // SparkFun BNO08x Breakout (Qwiic) defaults to 0x4B
//#define BNO08X_ADDR 0x4A // Alternate address if ADR jumper is closed

// For Battery voltage monitor
#include <movingAvg.h>
const int batteryMonitorPin = A5;  // Analog input connected to battery monitor circuit

// Constants
#define LOOP_DELAY_MS 10
#define BATTERY_PUB_RATE_MS 1000 // ms
#define BATTERY_SAMPLE_RATE_MS (BATTERY_PUB_RATE_MS / 10)
#define SENSOR_PUB_RATE_MS  50 // ms
#define STRIP_BLINK_RATE_MS 500 // ms

#define NEOPIXEL_STRIP_PIN MOSI     // Pin M0 on board, Supports DMA
#define NEOPIXEL_STRIP_LENGTH 8
#define STRIP_MODE_OFF 0
#define STRIP_MODE_NORMAL 1
#define STRIP_MODE_RAINBOW 2
#define STRIP_MODE_RANDOM 3


const int DEBUG_STRING_LEN            = 80;
const int STATUS_STRING_LEN           = 80;
const int NUMBER_OF_JOYSTICK_AXIS     = 12;
const int NUMBER_OF_JOYSTICK_BUTTONS  = 12;
const int WIFI_SSID_LEN               = 40;
const int WIFI_PW_LEN                 = 40;
const int STATUS_ITEM_STR_LEN         = 20;

/////////////////////////////////////////////////////////////////////////////////////
// Global Variables

ros::NodeHandle nh;

String            debugString;
//String            systemStatusString;
char              debugStringChar[DEBUG_STRING_LEN];
char              systemStatusChar[STATUS_STRING_LEN];
char              wifi_ssid_char[WIFI_SSID_LEN];
char              wifi_pw_char[WIFI_PW_LEN];

float             JoyStickAxis[NUMBER_OF_JOYSTICK_AXIS];        // Array of Axis the Joystick can set
long              JoyStickButtons[NUMBER_OF_JOYSTICK_BUTTONS];  // Array of Buttons the Joystick can set
int               button_number;

bool HeartBeatLedState = false;
uint32_t onboardNeoPixelColor = 0;
const uint8_t onboardNeoPixelIntensity = 32;  // 0 - 255
bool IMU_found = false;
int strip_mode = 1; // Normal mode

int  phone_imu_bytes_pending = 0;
int  bt_pitch = 0; // raw values from phone bluetooth serial
int  bt_roll = 0;
int  bt_azimuth = 0;
bool phone_wifi_get_ssid = false;
bool phone_wifi_get_pw = false;
int  wifi_char_index = 0;

int battery_sample_time = 0;
int battery_publish_time = 0;
int sensor_publish_time = 0;
int strip_blink_time = 0;

movingAvg BatteryAvg(10);  // Number of samples
float battery_previous_value = 0;
movingAvg IR0Avg(10);  // Sharp IR ranger, Number of samples
movingAvg IR1Avg(10);  
movingAvg US0Avg(10);  // Ultrasonic ranger

// Use DMA NeoPixel Library
Adafruit_NeoPixel_ZeroDMA Neopixel_Strip = Adafruit_NeoPixel_ZeroDMA(NEOPIXEL_STRIP_LENGTH, NEOPIXEL_STRIP_PIN, NEO_GRB + NEO_KHZ800);

#ifdef FEATHER_M4_EXPRESS
// enable the single onboard Neopixel
#define NUMBER_OF_ONBOARD_NEOPIXELS 1
#define ONBOARD_NEOPIXEL_PIN 8
#define ONBOARD_NEOPIXEL_INDEX 0
Adafruit_NeoPixel onboard_neopixel = Adafruit_NeoPixel(NUMBER_OF_ONBOARD_NEOPIXELS, ONBOARD_NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
#endif


////////////////////////////////////////////////////////////////////////////////
// ROS node subscribers and publishers

// PUBLISHERS
geometry_msgs::Point32 imuOrientationMsg;                                    // from BNO086 IMU
ros::Publisher pub_imu_orientation("/imu_orientation", &imuOrientationMsg);  // Robot body pose in x,y,z

std_msgs::Float32 batteryMsg;  // Monitor voltage of attached lipo battery
ros::Publisher pub_battery("/battery_voltage", &batteryMsg);

std_msgs::UInt16 batteryRawMsg;  // Monitor voltage of attached lipo battery
ros::Publisher pub_battery_raw("/battery_raw", &batteryRawMsg);

std_msgs::Float32 rangeMsg;  // Distance to target from range sensors
ros::Publisher pub_ir_range_right("/sensor_range/ir_right", &rangeMsg);
ros::Publisher pub_ir_range_left("/sensor_range/ir_left", &rangeMsg);
ros::Publisher pub_us_range_rear("/sensor_range/us_rear", &rangeMsg);

sensor_msgs::Joy bt_motor_cmd_msg;
ros::Publisher pub_bluetooth_motor_cmd("/phone_joy", &bt_motor_cmd_msg);

behavior_msgs::CommandState behavior_cmd_msg;
ros::Publisher pub_behavior_cmd("/behavior/cmd", &behavior_cmd_msg);
ros::Publisher pub_wifi_connect("/wifi_connect", &behavior_cmd_msg); // send this command just to the wifi connection node

system_status_msgs::SystemStatus system_status_msg;
ros::Publisher pub_system_status("/system_status", &system_status_msg);

// SUBSCRIBERS AND CALLBACK FUNCTIONS

// Use special message name ("/phone_update") to avoid hanging phone with too much data
// Status to send back to bluetooth phone
void system_status_callback(const system_status_msgs::SystemStatus& status_msg) {
  // send status back to bluetooth phone
  // Start by padding the end of the item, so the status strings line up
  /*
  String itemString = String(status_msg.item);
  int item_len = itemString.length();
  for (int i = item_len; i < STATUS_ITEM_STR_LEN; i++) {
    itemString += " ";
  }
  String systemStatusString = itemString + String(status_msg.status);
  */
  String systemStatusString = String(status_msg.item) + " : " + String(status_msg.status);
  systemStatusString.toCharArray(systemStatusChar, STATUS_STRING_LEN );
  Serial1.println(systemStatusChar);

}
ros::Subscriber<system_status_msgs::SystemStatus> SystemStatusSubscriber("/phone_update", &system_status_callback);


// LED STRIP COLOR COMMANDS  First byte is the leds to set (in bits)
// To test, try this: rostopic pub -1 /body/strip_color std_msgs/UInt32 0x01002f2f (second led, green/blue)

void set_strip_leds(const std_msgs::UInt32& cmd_msg) { 
  // Utility function for the callbacks
  uint32_t color_cmd = cmd_msg.data;
  uint32_t strip_color = color_cmd & 0xFFFFFF;
  uint32_t ledsToSet = (color_cmd >> 24) & 0xFF;
  for(int i = 0; i < 8; i++) {  // max of 8 leds. if more needed, change std_msgs::UInt32 to UInt64
    if ((ledsToSet & 1) == 1) {
      // set this led
      Neopixel_Strip.setPixelColor(i, strip_color);
    }
    ledsToSet =  ledsToSet >> 1; // shift to next bit
    Neopixel_Strip.show();
  }
}

void strip_color_callback(const std_msgs::UInt32& cmd_msg) {
  // Set leds according to specified color
  if(STRIP_MODE_NORMAL == strip_mode) {
    set_strip_leds(cmd_msg);
  }
}
ros::Subscriber<std_msgs::UInt32> stripColorSubscriber("/body/strip_color", &strip_color_callback);


// LED STRIP MODE command: To test, try this: rostopic pub -1 /body/strip_mode std_msgs/UInt16  1 // 1=Normal Mode
void strip_mode_callback(const std_msgs::UInt16& cmd_msg) {
  strip_mode = cmd_msg.data;

  if(STRIP_MODE_OFF == strip_mode) { // No longer used. Just causes lots of bugs
    neopixel_strip_solid(Neopixel_Strip.Color(0, 0, 0)); // Just Turn LEDs off until next color command
    nh.loginfo("Body Arduino: LED MODE COMMAND = OFF IGNORED");
    strip_mode = STRIP_MODE_NORMAL;
  }
  else if(STRIP_MODE_NORMAL == strip_mode) {
    neopixel_strip_solid(Neopixel_Strip.Color(0, 0, 0)); // Clear all LEDs when switching modes
    nh.loginfo("Body Arduino: LED MODE COMMAND = NORMAL");

  }
  else if(STRIP_MODE_RAINBOW == strip_mode) {
    nh.loginfo("Body Arduino: LED MODE COMMAND = RAINBOW");
    int delay = 50; // Tune this for effect you want
    theaterChaseRainbow(delay); // Rainbow-enhanced theaterChase variant
    neopixel_strip_solid(Neopixel_Strip.Color(0, 0, 0)); // Turn LEDs off when done
    strip_mode = STRIP_MODE_NORMAL; // Return to normal mode when done with this effect

  }
  else if(STRIP_MODE_RANDOM == strip_mode) {
    nh.loginfo("Body Arduino: LED MODE COMMAND = RANDOM");
    strip_blink_time = 0; // start timer    
  }
  else {
    nh.loginfo("Body Arduino: LED MODE COMMAND = UNKNOWN (set to normal)");
    strip_mode = STRIP_MODE_NORMAL; 
 }
}
ros::Subscriber<std_msgs::UInt16> stripModeSubscriber("/body/strip_mode", &strip_mode_callback);


// STRIP RAINBOW COMMAND  value is delay in ms between steps
// To test, try this: rostopic pub -1 /body/strip_rainbow std_msgs/UInt32 50
// TODO - make this work for multiple leds at once?
void strip_rainbow_callback(const std_msgs::UInt32& cmd_msg) {
  int delay = cmd_msg.data;
  nh.loginfo("Body Arduino: Strip Rainbow");  
  theaterChaseRainbow(delay); // Rainbow-enhanced theaterChase variant
  neopixel_strip_solid(Neopixel_Strip.Color(0, 0, 0)); // Turn LEDs off when done

}
ros::Subscriber<std_msgs::UInt32> stripRainbowSubscriber("/body/strip_rainbow", &strip_rainbow_callback);



////////////////////////////////////////////////////////////////////////////////
void setup() {

  pinMode(LED_BUILTIN, OUTPUT);  // heartbeat LED
  // randomSeed(analogRead(0));
#ifdef FEATHER_M4_EXPRESS  // Feather M4 has a neopixel we can blink
  onboard_neopixel.begin();
  onboard_neopixel.setPixelColor(ONBOARD_NEOPIXEL_INDEX, 63, 63, 63);  // pixel index, color
  onboard_neopixel.show();
#endif

  // Initialize ROS
  nh.initNode();
  nh.loginfo("Body Arduino: ROS Init Start");
  nh.advertise(pub_imu_orientation);
  nh.advertise(pub_battery);
  nh.advertise(pub_battery_raw);
  nh.advertise(pub_ir_range_right);
  nh.advertise(pub_ir_range_left);
  nh.advertise(pub_us_range_rear);
  nh.advertise(pub_bluetooth_motor_cmd);
  nh.advertise(pub_behavior_cmd);
  nh.advertise(pub_wifi_connect);
  nh.advertise(pub_system_status);

  nh.subscribe(SystemStatusSubscriber);
  nh.subscribe(stripModeSubscriber);
  nh.subscribe(stripColorSubscriber);
  nh.subscribe(stripRainbowSubscriber);
  
  // blink LED on the board at startup
  for (int i = 0; i < 10; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }

  // Initialize IMU
  Wire.begin();
  BatteryAvg.begin();
  IR0Avg.begin();
  IR1Avg.begin();
  US0Avg.begin();
  Neopixel_Strip.begin();
  Neopixel_Strip.show(); // Initialize all pixels to 'off'

  Serial1.begin(115200);    // connection with bluetooth phone


#ifdef FEATHER_M4_EXPRESS  // Feather M4 has a neopixel we can blink
  //onboard_neopixel.setPixelColor(ONBOARD_NEOPIXEL_INDEX, 127, 0, 0);  // pixel index, color
  //onboard_neopixel.show();
  //delay(200);

  onboard_neopixel.setPixelColor(ONBOARD_NEOPIXEL_INDEX, 63, 63, 63);
  onboard_neopixel.show();
  neopixel_strip_solid(Neopixel_Strip.Color(63, 63, 63)); //GBR
  delay(200);

  onboard_neopixel.setPixelColor(ONBOARD_NEOPIXEL_INDEX, 0, 0, 63);
  onboard_neopixel.show();
  neopixel_strip_solid(Neopixel_Strip.Color(0, 0, 63)); //GBR
  delay(200);

  onboard_neopixel.setPixelColor(ONBOARD_NEOPIXEL_INDEX, 0, 63, 0);
  onboard_neopixel.show();
  neopixel_strip_solid(Neopixel_Strip.Color(0, 63, 0)); //GBR
  delay(200);

  onboard_neopixel.setPixelColor(ONBOARD_NEOPIXEL_INDEX, 63, 0, 0);
  onboard_neopixel.show();
  neopixel_strip_solid(Neopixel_Strip.Color(63, 0, 0)); //GBR
  delay(200);

  // Set pixel to Red before trying I2C IMU
  onboard_neopixel.setPixelColor(ONBOARD_NEOPIXEL_INDEX, 63, 0, 0);
  onboard_neopixel.show();
  neopixel_strip_solid(Neopixel_Strip.Color(0, 0, 0)); // Turn LEDs off
  delay(100);
#endif


  // TEST NEOPIXEL STRIP
  //rainbow(10);
  theaterChaseRainbow(50); // Rainbow-enhanced theaterChase variant
  neopixel_strip_solid(Neopixel_Strip.Color(0, 0, 0)); // Turn LEDs off

  // DEBUG: TEST FUNCTION
  /***
  std_msgs::UInt32 tmp_cmd_msg;
  tmp_cmd_msg.data = 0xFFFF0000;
  set_strip_leds(tmp_cmd_msg);
  delay(500);

  tmp_cmd_msg.data = 0xFF00FF00;
  set_strip_leds(tmp_cmd_msg);
  delay(500);

  tmp_cmd_msg.data = 0xFF0000FF;
  set_strip_leds(tmp_cmd_msg);
  delay(500);

  tmp_cmd_msg.data = 0x00000000;
  set_strip_leds(tmp_cmd_msg);
  delay(500);

  neopixel_strip_solid(Neopixel_Strip.Color(0, 0, 0)); // Turn LEDs off immediately

  ***/


  /*
  neopixel_strip_solid(Neopixel_Strip.Color(255, 0, 0)); //GBR
  delay(500);
  neopixel_strip_solid(Neopixel_Strip.Color(0, 255, 0)); //GBR
  delay(500);
  neopixel_strip_solid(Neopixel_Strip.Color(0, 0, 255)); //GBR
  delay(500);
  */

  // Initialize IMU
  IMU_found = myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST);
  if (!IMU_found) {
    nh.logwarn("WARNING! BNO08x IMU not detected!");
  } else {
    nh.loginfo("Body Arduino: BNO08x found!");
  }

  // Wire.setClock(400000); //Increase I2C data rate to 400kHz
  setReports();

  // Bluetooth phone as a Joystick
  bt_motor_cmd_msg.axes = JoyStickAxis;
  bt_motor_cmd_msg.axes_length = NUMBER_OF_JOYSTICK_AXIS;
  bt_motor_cmd_msg.axes[0] = 0.0;
  bt_motor_cmd_msg.axes[1] = 0.0;

  bt_motor_cmd_msg.buttons = JoyStickButtons;
  bt_motor_cmd_msg.buttons_length = NUMBER_OF_JOYSTICK_BUTTONS;
  for ( int i=0; i < NUMBER_OF_JOYSTICK_BUTTONS; i++ ) {
    bt_motor_cmd_msg.buttons[i] = 0;
  }




  nh.loginfo("Body Arduino: started");
}


////////////////////////////////////////////////////////////////////////////////
void loop() {

  // nh.loginfo("Body Arduino: LOOP");

#ifdef FEATHER_M4_EXPRESS
  onboard_neopixel.setPixelColor(ONBOARD_NEOPIXEL_INDEX, 63, 0, 0);  // Set pixel to Red before trying I2C IMU
  onboard_neopixel.show();
#endif

  // Get IMU Updates
  if (myIMU.wasReset()) {
    //Serial.print("IMU BNO086 Sensor was reset!");
    setReports();
  }

  // Has a new event come in on the Sensor Hub Bus?
  if (myIMU.getSensorEvent() == true) {

    // is it the correct sensor data we want?
    if (myIMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {

      // Match board orientation inside robot:

      imuOrientationMsg.x = (myIMU.getPitch()) * 180.0 / PI;  // Convert pitch to degrees
      imuOrientationMsg.y = (myIMU.getRoll()) * 180.0 / PI;   // Convert roll to degrees
      imuOrientationMsg.z = (myIMU.getYaw()) * 180.0 / PI;    // Convert yaw / heading to degrees

      // if( (0.0 != imuOrientationMsg.x) || (0.0 != imuOrientationMsg.y) || (0.0 != imuOrientationMsg.z) ) {
      // wait for the IMU to be ready before publishing?
      pub_imu_orientation.publish(&imuOrientationMsg);
    }
  }

  // Read distance sensors every loop, and average values to reduce noise
  int a2d_data = analogRead(A0); // IR Rangers
  IR0Avg.reading(a2d_data);
 
  a2d_data = analogRead(A1);
  IR1Avg.reading(a2d_data);

  a2d_data = analogRead(A4); // UltraSonic Ranger
  US0Avg.reading(a2d_data);

  //////////////////////////////////////////////////////////////
  // Bluetooth commands from Phone
  // Read Bluetooth serial. This code is a bit funky, because I wanted to pack each IMU axis into a single byte.
  // Buttons are single byte with the button number. 
  // IMU is 4 bytes: [button_number = 255], [Pitch] [Roll] [Compass] in Degrees, with 0 = straight up, 180 = down
  button_number = 0;
  while (Serial1.available() > 0) {  // Checks whether data is coming from the serial port
    int bt_value = Serial1.read();   // Reads one byte of data from the serial port
 
    // Handle bluetooh phone WiFi connect request
    if (bt_value == 252) { // Magic number to indicate wifi connect request
      nh.loginfo("BODY ARDUINO: Bluetooth WiFi Connect Request");
      phone_wifi_get_ssid = true;
      phone_wifi_get_pw = false;
      wifi_char_index = 0;
      wifi_ssid_char[0] = '\0';
      continue;

    }
    else if (phone_wifi_get_ssid) {
      if (bt_value > 126) {
        // not an ascii value. Abort!
        nh.loginfo("BODY ARDUINO: Bluetooth WiFi: SSID Bad Data!");
        phone_wifi_get_ssid = false;
      }
      else if (bt_value == ':') { // Separator token
        // Done getting SSID
        wifi_ssid_char[wifi_char_index] = '\0';
        phone_wifi_get_ssid = false;
        phone_wifi_get_pw = true;
        wifi_char_index = 0;
        nh.loginfo("BODY ARDUINO: Bluetooth WiFi Got SSID");
      }
      else {
        wifi_ssid_char[wifi_char_index] = bt_value;
        wifi_char_index++;
      }

    }
    else if (phone_wifi_get_pw) {
      if (bt_value > 126) {
        // not an ascii value. Abort!
        nh.loginfo("BODY ARDUINO: Bluetooth WiFi: PW Bad Data!");
        phone_wifi_get_pw = false;
      }
      else if ((bt_value == ':') || (bt_value == '\0'))  { // Separator token or end of string
        // Done getting PW
        wifi_pw_char[wifi_char_index] = '\0';
        phone_wifi_get_pw = false;
        wifi_char_index = 0;
        nh.loginfo("BODY ARDUINO: Bluetooth WiFi Got PW. Sending Command.");
        debugString = String("BODY ARDUINO: SSID: ") + String(wifi_ssid_char) + String(" PW: ") + String(wifi_pw_char);
        debugString.toCharArray(debugStringChar, DEBUG_STRING_LEN );
        nh.loginfo(debugStringChar);
 
        // Send the command!
        behavior_cmd_msg.commandState = "WIFI_CONNECT"; 
        behavior_cmd_msg.param1 = wifi_ssid_char;
        behavior_cmd_msg.param2 = wifi_pw_char;
        pub_wifi_connect.publish( &behavior_cmd_msg ); // send this command just to the wifi connection node
        nh.loginfo("BODY ARDUINO: Bluetooth WiFi Command sent, doing cleanup");

        // clear out the command
        behavior_cmd_msg.commandState = ""; 
        behavior_cmd_msg.param1 = "";
        behavior_cmd_msg.param2 = "";
        nh.loginfo("BODY ARDUINO: Bluetooth WiFi connect: Done sending command.");

      }
      else {
        wifi_pw_char[wifi_char_index] = bt_value;
        wifi_char_index++;
      }
    }

    // Handle bluetooth phone ROS Launch message
    else if (bt_value == 251) { // Magic number to indicate ROS Launch request
        behavior_cmd_msg.commandState = "ROS_START"; 
        behavior_cmd_msg.param1 = "X";
        behavior_cmd_msg.param2 = "X";
        pub_wifi_connect.publish( &behavior_cmd_msg ); // send this command just to the wifi connection node, which also starts ROS
        nh.loginfo("BODY ARDUINO: Bluetooth ROS START Command sent, doing cleanup");

        // clear out the command
        behavior_cmd_msg.commandState = ""; 
        behavior_cmd_msg.param1 = "";
        behavior_cmd_msg.param2 = "";
        nh.loginfo("BODY ARDUINO: Bluetooth ROS START: Done sending command.");

    } 

    // Check for IMU / Accelerometer message from the phone
    else if (bt_value == 255) { // Magic number to separate IMU message from keypress message
      // give time for 3 data bytes to be sent
      phone_imu_bytes_pending = 3;
      continue;
    }
    else if (phone_imu_bytes_pending == 3) {
      phone_imu_bytes_pending = 2;
      bt_pitch = bt_value - 32; // values are shifted on phone to assure no overlap with button numbers
      // debugString = String("BODY ARDUINO: Raw Pitch: ") + String(bt_pitch);
      // debugString.toCharArray(debugStringChar, DEBUG_STRING_LEN );
      // nh.loginfo(debugStringChar);
      continue;
    }
    else if (phone_imu_bytes_pending == 2) {
      phone_imu_bytes_pending = 1;
      bt_roll = bt_value - 32;
      // debugString = String("BODY ARDUINO: Raw Roll: ") + String(bt_roll);
      // debugString.toCharArray(debugStringChar, DEBUG_STRING_LEN );
      // nh.loginfo(debugStringChar);
      continue;
    }
    else if (phone_imu_bytes_pending == 1) {
      phone_imu_bytes_pending = 0;
      bt_azimuth = bt_value - 32;

      // Got all values!
      // Convert from phone raw values with correct offset and direction
      // final values should be in the range of -1.0 to +1.0 for motor commands
      float pitch_scaled = ((float)bt_pitch - 90.0) / 90.0;
      float roll_scaled = ((float)bt_roll - 90.0) / 90.0;
      // azimuth not used
      //float azimuth = bt_azimuth * 10.0;       // Sent as 1/10 360 degree value to fit in a byte (and accuracy not important)

      // Apply a Deadzone (values are percents, with 1.0 = 100%)
      // Tune these for most natural control
      if ((pitch_scaled <= 0.10) && (pitch_scaled >= -0.200)) {
        pitch_scaled = 0.0;
      }
      else if (pitch_scaled > 0.10) {
        pitch_scaled = pitch_scaled - 0.10; // Scale forward to start at this value
      }
      else if (pitch_scaled < -0.30) {
        pitch_scaled = pitch_scaled + 0.20; // Scale backup to start at this value
      }

      // Turn / Roll Deadzone
      if ((roll_scaled <= 0.05) && (roll_scaled >= -0.05)) {
        roll_scaled = 0.0;
      }
      else if (roll_scaled > 0.05) {
        roll_scaled = roll_scaled - 0.05; // Scale turns to start at this value
      }
      else if (roll_scaled < -0.05) {
        roll_scaled = roll_scaled + 0.05; 
      }


      //nh.loginfo("BODY ARDUINO: Phone ACC Update");
      // publish IMU messages as Joystick messages
      bt_motor_cmd_msg.buttons[4] = 1;  // Enable Deadman Switch      
      bt_motor_cmd_msg.axes[1] = pitch_scaled;
      bt_motor_cmd_msg.axes[0] = roll_scaled;
      bt_motor_cmd_msg.header.stamp = nh.now();
      pub_bluetooth_motor_cmd.publish(&bt_motor_cmd_msg);
      //debugString = String(BODY ARDUINO: BlueTooth Pitch: ") + String(pitch_scaled) + String(" Roll: ") + String(roll_scaled);
      //debugString.toCharArray(debugStringChar, DEBUG_STRING_LEN );
      //nh.loginfo(debugStringChar);
      continue;

    
    } 
    // Handle bluetooth phone Connected and Disconnected messages
    else if (bt_value == 254) { // Magic number to indicate phone connected
      nh.loginfo("BODY ARDUINO: Bluetooth Phone CONNECTED");
      system_status_msg.item = "BLUETOOTH_PHONE";
      system_status_msg.status = "CONNECTED";
      pub_system_status.publish(&system_status_msg);
      continue;

    } 
    else if (bt_value == 253) { // Magic number to indicate phone disconnected (may not always arrive)
      nh.loginfo("BODY ARDUINO: Bluetooth Phone DISCONNECTED");
      // Force stop, incase user accidentally had joystick mode on.
      bt_motor_cmd_msg.buttons[4] = 1;  // Enable Deadman Switch      
      bt_motor_cmd_msg.axes[0] = 0.0;
      bt_motor_cmd_msg.axes[1] = 0.0;
      bt_motor_cmd_msg.header.stamp = nh.now();
      pub_bluetooth_motor_cmd.publish(&bt_motor_cmd_msg);
      system_status_msg.item = "WIFI_CONNECT";
      system_status_msg.status = "DISCONNECTED";
      pub_system_status.publish(&system_status_msg);
      continue;


    } 
    else {
      
      // Not special message, just a keypress
      button_number = bt_value;
      behavior_cmd_msg.param1 = "";
      behavior_cmd_msg.param2 = "";

      // Send bluetooth command to behavior controler
      switch (button_number) {

        case 1: // Key 1 Toggle Off (also see key 17 below)
          // IMU/Accelerometer disabled. Send stop as last IMU command.
          nh.loginfo("BODY ARDUINO: Bluetooth Phone Joystick mode DISABLED");
          bt_motor_cmd_msg.buttons[4] = 1;  // Enable Deadman Switch      
          bt_motor_cmd_msg.axes[0] = 0.0;
          bt_motor_cmd_msg.axes[1] = 0.0;
          bt_motor_cmd_msg.header.stamp = nh.now();
          pub_bluetooth_motor_cmd.publish(&bt_motor_cmd_msg);
          continue; // Don't send a key command

        case 2:
          behavior_cmd_msg.commandState = "SAY"; 
          behavior_cmd_msg.param1 = "HELLO";
          break;
        case 3:
          behavior_cmd_msg.commandState = "HEAD_CENTER";
          break;
        case 4:
          behavior_cmd_msg.commandState = "HAPPY"; // Do a little happy dance
          break;
        case 5:
          // FORCE STOP
          bt_motor_cmd_msg.buttons[4] = 1;  // Enable Deadman Switch      
          bt_motor_cmd_msg.axes[0] = 0.0;
          bt_motor_cmd_msg.axes[1] = 0.0;
          bt_motor_cmd_msg.header.stamp = nh.now();
          pub_bluetooth_motor_cmd.publish(&bt_motor_cmd_msg);
          behavior_cmd_msg.commandState = "STOP";
          break;
        case 6:
          behavior_cmd_msg.commandState = "INTRO";
          break;
        case 7:
          behavior_cmd_msg.commandState = "TELL_JOKE";
          behavior_cmd_msg.param1 = "STAR WARS"; 
          break;
        case 8: // button 8 is a toggle
          behavior_cmd_msg.commandState = "AI_MODE"; 
          behavior_cmd_msg.param1 = "TOGGLE";
          break;
        case 9:
          behavior_cmd_msg.commandState = "POSE"; 
          behavior_cmd_msg.param1 = "UP";
          behavior_cmd_msg.param2 = "0.3";
          break;
        case 10:
          behavior_cmd_msg.commandState = "DANGER";
          break;
        case 11:
          behavior_cmd_msg.commandState = "PEEK"; 
          break;
        case 12:
          behavior_cmd_msg.commandState = "BOW"; // Currently " - " on phone
          break;
        case 13:
          behavior_cmd_msg.commandState = "POSE"; 
          behavior_cmd_msg.param1 = "DOWN";
          behavior_cmd_msg.param2 = "0.3";
          break;
        case 14:
          // Default Song, but should never happen! See song selection starting at 21
          behavior_cmd_msg.commandState = "DANCE";  
          break;
       case 15:
          behavior_cmd_msg.commandState = "WAKEUP";
          break;
        case 16:
          behavior_cmd_msg.commandState = "SLEEP";
          break;
        
        case 17: 
          // Key 1 Toggle On (just informational)
          // IMU/Accelerometer enabled. No action needed here.
          nh.loginfo("BODY ARDUINO: Bluetooth Phone Joystick mode ENABLED");
          continue; // Don't send a key command

        // Song selectons
        case 21:
          behavior_cmd_msg.commandState = "DANCE";  // First song in list
          behavior_cmd_msg.param1 = "1";
          break;
        case 22:
          behavior_cmd_msg.commandState = "DANCE"; 
          behavior_cmd_msg.param1 = "2";
          break;
        case 23:
          behavior_cmd_msg.commandState = "DANCE"; 
          behavior_cmd_msg.param1 = "3";
          break;
        case 24:
          behavior_cmd_msg.commandState = "DANCE"; 
          behavior_cmd_msg.param1 = "4";
          break;
        case 25:
          behavior_cmd_msg.commandState = "DANCE"; 
          behavior_cmd_msg.param1 = "5";
          break;
        case 26:
          behavior_cmd_msg.commandState = "DANCE"; 
          behavior_cmd_msg.param1 = "6";
          break;
        case 27:
          behavior_cmd_msg.commandState = "DANCE"; 
          behavior_cmd_msg.param1 = "7";
          break;
        case 28:
          behavior_cmd_msg.commandState = "DANCE"; 
          behavior_cmd_msg.param1 = "8";
          break;
        case 29:
          behavior_cmd_msg.commandState = "DANCE"; 
          behavior_cmd_msg.param1 = "9";
          break;


        default:
          debugString = String("BODY ARDUINO: UNHANDLED PHONE COMMAND BUTTON NUMBER: ") + String(button_number);
          debugString.toCharArray(debugStringChar, DEBUG_STRING_LEN );
          nh.logwarn(debugStringChar);
          button_number = 0; // Don't send a key command
          behavior_cmd_msg.commandState = "UNHANDLED";
          
      }

      if (button_number != 0) {
        // New command received from phone
        // nh.loginfo(BODY ARDUINO: Phone Command Received.  Key = ");
        // debugString = String(button_number);
        // debugString.toCharArray(debugStringChar, 10 );
        // nh.loginfo(debugStringChar);
      
        debugString = String("BODY ARDUINO: Sending Command: ") + String(behavior_cmd_msg.commandState);
        debugString.toCharArray(debugStringChar, DEBUG_STRING_LEN );
        nh.loginfo(debugStringChar);
          
        pub_behavior_cmd.publish( &behavior_cmd_msg );

      }

    } 

  } // end of while serial available, get data from bluetooth phone




  // Onboard Neopixel Heartbeat
#ifdef FEATHER_M4_EXPRESS
  onboard_neopixel.setPixelColor(ONBOARD_NEOPIXEL_INDEX, 0, 0, 63);  // Set pixel to blue for battery and sleep time
  onboard_neopixel.show();
#endif

  if ((sensor_publish_time += LOOP_DELAY_MS) > SENSOR_PUB_RATE_MS) {
    // Time to publish sensor readings (about every 100ms)
    // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 3.3V):
    // then convert Sharp GP2Y0A02YK IR output voltage to distance. Linear approx is good enough.
    // If you want more accuracy, try https://www.omnicalculator.com/statistics/polynomial-regression
    // distance = (22.5596 * pow(voltage, 2)) - (108.4983 * voltage) + 152.2292;
    // distance = (-20.5148 * pow(voltage, 3)) + (121.561 * pow(voltage, 2)) - (254.0702 * voltage) + 216.7491;

    // First IR sensor
    float sensor_avg = IR0Avg.getAvg();
    float voltage = float(sensor_avg) * (3.3 / 1023.0);
    float distance = ((-36.439 * voltage) + 104.102) / 100.0;
    rangeMsg.data = distance; // in meters
    pub_ir_range_right.publish(&rangeMsg);

    // Now the other IR sensor
    sensor_avg = IR1Avg.getAvg();
    voltage = float(sensor_avg) * (3.3 / 1023.0);
    distance = ((-36.439 * voltage) + 104.102) / 100.0;  // Linear approx is good enough.
    rangeMsg.data = distance; // in meters
    pub_ir_range_left.publish(&rangeMsg);

  // Now the ultrasonic sensor
    sensor_avg = US0Avg.getAvg();
    voltage = float(sensor_avg) * (3.3 / 1023.0);
    // voltage converted to inches then meters, and offset 0.020 for position in robot
    distance = (voltage * (512/3.3) * .0254) - 0.020; 
    rangeMsg.data = distance;
    pub_us_range_rear.publish(&rangeMsg); // useful range: 18cm to 170cm

    sensor_publish_time = 0;

  }

  if ((battery_sample_time += LOOP_DELAY_MS) > BATTERY_SAMPLE_RATE_MS) {
    // read the battery voltage and average it
    int battery_a2d = analogRead(batteryMonitorPin);
    BatteryAvg.reading(battery_a2d);
    battery_sample_time = 0;

  }

  // Publish battery voltage every n ms
  if ((battery_publish_time += LOOP_DELAY_MS) > BATTERY_PUB_RATE_MS) {

    int battery_a2d_avg = BatteryAvg.getAvg(); 

    ///////////////////////////////////////////////
    // DEBUG FOR GETTING RAW READINGS
    //uint raw_battery_value = abs(battery_a2d_avg);
    //batteryRawMsg.data = raw_battery_value;
    //pub_battery_raw.publish(&batteryRawMsg);

    ///////////////////////////////////////////////

    // Calculate and publish battery voltage
    // Y =  mx + b 
    // mx = 0.007, b = 9.8145

    float calculatedVoltage = (0.007 * (float)battery_a2d_avg) + 9.8145;
    if (calculatedVoltage < 0.0) {
      calculatedVoltage = 0.0;
    }

    if (abs(calculatedVoltage - battery_previous_value) < 1.0) {  // volt change per second
      // this will skip sudden changes to allow settling, like when battery unplugged
      batteryMsg.data = calculatedVoltage;
      pub_battery.publish(&batteryMsg);
    }
    battery_previous_value = calculatedVoltage;
    battery_publish_time = 0;
 
  }


  // LED RANDOM - Randomly blink LEDs on front of robot body (if enabled)
  if (strip_mode == STRIP_MODE_RANDOM) {
    if ((strip_blink_time += LOOP_DELAY_MS) > STRIP_BLINK_RATE_MS) {
      strip_blink_time = 0;

      // do random led updates on body neopixels
      // colors are constrained to 9 combinations of r,g,b values at constant brightness
      int rand_delay = random(0,100);
      if (rand_delay > 30) {                 // Adjust how much jitter here
        for(int led_position=0; led_position < 8; led_position++) {
          uint32_t led_color = 0;
          int rgb_color_bits = random(0,8);  // make this value larger than 6 to have more off time on each LED
          switch (rgb_color_bits) {
            case 0:
              led_color = 0x000000;
              break;
            case 1:
              led_color = 0x00001F;
              break;
            case 2:
              led_color = 0x001F00;
              break;
            case 3:
              led_color = 0x1F0000;
              break;
            case 4:
              led_color = 0x000F0F;
              break;
            case 5:
              led_color = 0x0F0F00;
              break;
            case 6:
              led_color = 0x0F000F;
              break;

            default:
              led_color = 0x000000; 
              break;
          }
          Neopixel_Strip.setPixelColor(led_position, led_color);

        }

        Neopixel_Strip.show();
      }
    }
  } 


  // nh.loginfo("Body Arduino:  Loop");
  nh.spinOnce();  // ROS Heartbeat / Communicate with ROS
  delay(LOOP_DELAY_MS);
}

////////////////////////////////////////////////////////////////////////////////
// UTILITIES

// Define the sensor outputs to receive
void setReports(void) {

  nh.loginfo("Body Arduino: Setting desired IMU reports");
  if (myIMU.enableRotationVector() == true) {
    nh.loginfo("Body Arduino: Rotation vector enabled");
    nh.loginfo("Body Arduino: Output in form roll, pitch, yaw");
  } else {
    nh.loginfo("Body Arduino: Could not enable rotation vector");
  }
}

void neopixel_strip_solid(uint32_t c) {
  for (uint16_t i = 0; i < Neopixel_Strip.numPixels(); i++) {
    Neopixel_Strip.setPixelColor(i, c);
  }
  Neopixel_Strip.show(); 
}

// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(int wait) {
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
    //Neopixel_Strip.rainbow(firstPixelHue);
    // Above line is equivalent to:
    Neopixel_Strip.rainbow(firstPixelHue, 1, 255, 100, true);
    Neopixel_Strip.show(); // Update strip with new contents
    delay(wait);  // Pause for a moment
  }
}

// Rainbow-enhanced theater marquee. Pass delay time (in ms) between frames.
void theaterChaseRainbow(int wait) {
  int firstPixelHue = 0;     // First pixel starts at red (hue 0)
  for(int a=0; a<30; a++) {  // Repeat 30 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      Neopixel_Strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of Neopixel_Strip in increments of 3...
      for(int c=b; c<Neopixel_Strip.numPixels(); c += 3) {
        // hue of pixel 'c' is offset by an amount to make one full
        // revolution of the color wheel (range 65536) along the length
        // of the Neopixel_Strip (Neopixel_Strip.numPixels() steps):
        int      hue   = firstPixelHue + c * 65536L / Neopixel_Strip.numPixels();
        uint32_t color = Neopixel_Strip.gamma32(Neopixel_Strip.ColorHSV(hue)); // hue -> RGB
        Neopixel_Strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      Neopixel_Strip.show();                // Update Neopixel_Strip with new contents
      delay(wait);                 // Pause for a moment
      firstPixelHue += 65536 / 90; // One cycle of color wheel over 90 frames
    }
  }
}

