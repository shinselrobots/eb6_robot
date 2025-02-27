/* EB Body Feather M4.  Currently handles:
   - IMU: Sparkfun BNO086
   - Battery Voltage via A2D pin
   - Wheel motors, controlled as servos (servo pulse to motor controller used)
*/

// If not using a Feather, you might need to comment out one or both of these lines:
#define FEATHER_M4_EXPRESS  // if using the M4 Express board
#define USE_USBCON          // NEEDED FOR CPU with Built-in USB.  ATmega32u4 - Feather 32u4, Feather M4, LEONARDO...

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Point32.h>
#include <ros/time.h>

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

// For wheel motor control (controlled as a servo)
// #include <Servo.h>

// For Sparkfun BNO086 IMU:
#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h"  // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BNO08x
BNO08x myIMU;
// Per Sparkfun recommendation and example code, IMU connected with Interrupt and Reset as follows:
#define BNO08X_INT A4
#define BNO08X_RST A5
#define BNO08X_ADDR 0x4B  // SparkFun BNO08x Breakout (Qwiic) defaults to 0x4B
//#define BNO08X_ADDR 0x4A // Alternate address if ADR jumper is closed

// Wheel Motors
#define DEVICE_ID_RIGHT 9
#define DEVICE_ID_LEFT 8
#define NUM_STATUS_BYTES 8
#define ODOM_PUB_RATE_MS 1000

// For Battery voltage monitor
#include <movingAvg.h>
const int batteryMonitorPin = A3;  // Analog input connected to battery monitor circuit

#define LOOP_DELAY_MS 400  //10
#define BATTERY_PUB_RATE_MS 1000
#define BATTERY_SAMPLE_RATE_MS (BATTERY_PUB_RATE_MS / 10)

/////////////////////////////////////////////////////////////////////////////////////
// Global Variables

ros::NodeHandle nh;
bool HeartBeatLedState = false;
uint32_t onboardNeoPixelColor = 0;
const uint8_t onboardNeoPixelIntensity = 32;  // 0 - 255
//uint32_t        test_counter = 0;
bool IMU_found = false;
int i2c_enabled = 0;

int SpeedCmdRight = 0;      // Percent commanded speed (-100 to +100)
int SpeedCurrentRight = 0;  // current speed sent to motor
int SpeedCmdLeft = 0;       // Percent commanded speed (-100 to +100)
int SpeedCurrentLeft = 0;   // current speed sent to motor
char inData[10];
int ch_index = 0;

int odom_publish_time = 0;

int battery_sample_time = 0;
int battery_publish_time = 0;
int battery_a2d_avg = 0;
float calculatedVoltage = 0.0;
movingAvg BatteryAvg(10);  // Number of samples
//Servo           ServoRight;   // Servo object to control motor
//Servo           ServoLeft;    // Servo object to control motor

int dbg_loop_count = 0;

#ifdef FEATHER_M4_EXPRESS
// enable the single onboard Neopixel
#define NUMBER_OF_ONBOARD_NEOPIXELS 1
#define ONBOARD_NEOPIXEL_PIN 8
#define ONBOARD_NEOPIXEL_INDEX 0
Adafruit_NeoPixel onboard_neopixel = Adafruit_NeoPixel(NUMBER_OF_ONBOARD_NEOPIXELS, ONBOARD_NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
#endif

////////////////////////////////////////////////////////////////////////////////
// ROS node subscribers and publishers


// WHEEL MOTOR COMMANDS: To test, try this: rostopic pub -1 /wheel_speed_right std_msgs/Int16  -- -50 // 50% reverse speed
void wheel_speed_right_callback(const std_msgs::Int16& cmd_msg) {
  SpeedCmdRight = cmd_msg.data;
  //nh.loginfo("Body Arduino: RIGHT MOTOR COMMAND RECEIVED");
}
ros::Subscriber<std_msgs::Int16> wheel_speed_right_subscriber("/wheel_speed_right", &wheel_speed_right_callback);

void wheel_speed_left_callback(const std_msgs::Int16& cmd_msg) {
  SpeedCmdLeft = cmd_msg.data * -1;  // Left motor is reversed compared to right
  //nh.loginfo("Body Arduino: LEFT MOTOR COMMAND RECEIVED");
}
ros::Subscriber<std_msgs::Int16> wheel_speed_left_subscriber("/wheel_speed_left", &wheel_speed_left_callback);

// BNO086 IMU Sensor publisher
geometry_msgs::Point32 imuOrientationMsg;
ros::Publisher pub_imu_orientation("/imu_orientation", &imuOrientationMsg);  // Robot body pose in x,y,z

// Monitor voltage of attached lipo battery
std_msgs::Float32 batteryMsg;
ros::Publisher pub_battery("/battery_voltage", &batteryMsg);

// Monitor voltage of attached lipo battery
std_msgs::Int32 OdomMsgRight;
ros::Publisher pub_odom_right("/odom_right", &OdomMsgRight);


// publish optional debug messages (rather than clutter up terminal with log messages)
std_msgs::String str_msg;
ros::Publisher debug_pub("/body_arduino_dbg", &str_msg);
char heartbeat[20] = "Loop";

////////////////////////////////////////////////////////////////////////////////////
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);  // heartbeat LED
  // randomSeed(analogRead(0));
#ifdef FEATHER_M4_EXPRESS  // Feather M4 has a neopixel we can blink
  onboard_neopixel.begin();
  onboard_neopixel.setPixelColor(ONBOARD_NEOPIXEL_INDEX, 0, 0, 0);  // pixel index, color
  onboard_neopixel.show();
#endif

  // Initialize ROS
  nh.initNode();
  nh.loginfo("Body Arduino: ROS Init Start");
  nh.advertise(pub_imu_orientation);
  nh.advertise(pub_battery);
  nh.advertise(pub_odom_right);
  nh.subscribe(wheel_speed_right_subscriber);
  nh.subscribe(wheel_speed_left_subscriber);

  nh.advertise(debug_pub);

  // blink LED on the board at startup
  for (int i = 0; i < 10; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }

  // Initialize I2C and battery monitor
  Wire.begin();
  BatteryAvg.begin();


#ifdef FEATHER_M4_EXPRESS  // Feather M4 has a neopixel we can blink
  //onboard_neopixel.setPixelColor(ONBOARD_NEOPIXEL_INDEX, 127, 0, 0);  // pixel index, color
  //onboard_neopixel.show();
  //delay(200);
  onboard_neopixel.setPixelColor(ONBOARD_NEOPIXEL_INDEX, 0, 63, 0);
  onboard_neopixel.show();
  delay(200);
  onboard_neopixel.setPixelColor(ONBOARD_NEOPIXEL_INDEX, 0, 0, 63);
  onboard_neopixel.show();

  delay(200);
  onboard_neopixel.setPixelColor(ONBOARD_NEOPIXEL_INDEX, 0, 63, 0);
  onboard_neopixel.show();
  delay(200);
  onboard_neopixel.setPixelColor(ONBOARD_NEOPIXEL_INDEX, 0, 0, 63);
  onboard_neopixel.show();
  delay(200);

  // Set pixel to Red before trying I2C IMU
  onboard_neopixel.setPixelColor(ONBOARD_NEOPIXEL_INDEX, 63, 0, 0);
  onboard_neopixel.show();
  delay(100);
#endif

  // Initialize IMU
  IMU_found = myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST);
  if (!IMU_found) {
    nh.logwarn("WARNING! BNO08x IMU not detected!");
  } else {
    nh.loginfo("Body Arduino: BNO08x found!");
  }
  // Wire.setClock(400000); //Increase I2C data rate to 400kHz
  setReports();
  onboard_neopixel.setPixelColor(ONBOARD_NEOPIXEL_INDEX, 0, 63, 0);  // set pixel green - good IMU
  onboard_neopixel.show();

}  // End of Setup


////////////////////////////////////////////////////////////////////////////////
void loop() {

  onboard_neopixel.setPixelColor(ONBOARD_NEOPIXEL_INDEX, 63, 0, 0);  // Set pixel to Red before trying I2C IMU
  onboard_neopixel.show();

  // Get IMU Updates
  if (myIMU.wasReset()) {
    nh.loginfo("Body Arduino: IMU BNO086 Sensor was reset!");
    setReports();
  }
  // IMU: Has a new event come in on the Sensor Hub Bus?
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
  onboard_neopixel.setPixelColor(ONBOARD_NEOPIXEL_INDEX, 0, 0, 63);  // Set pixel to blue after I2C IMU
  onboard_neopixel.show();
  //delay(400);
  // Publish debug heartbeat
  str_msg.data = heartbeat;
  debug_pub.publish(&str_msg);
  nh.loginfo("Body Arduino: Loop");

  // Handle wheel speed command changes
  // TODO: if needed, add slow ramp here

  if (SpeedCurrentRight != SpeedCmdRight) {
    // speed change commanded. Send new speed to motor control
    // send_speed_command(DEVICE_ID_RIGHT, SpeedCmdRight);
    SpeedCurrentRight = SpeedCmdRight;
  }

  if (SpeedCurrentLeft != SpeedCmdLeft) {
    // TODO send_speed_command(DEVICE_ID_LEFT, SpeedCmdLeft);
    SpeedCurrentLeft = SpeedCmdLeft;
  }

  if ((battery_sample_time += LOOP_DELAY_MS) > BATTERY_SAMPLE_RATE_MS) {
    int battery_a2d = analogRead(batteryMonitorPin);
    BatteryAvg.reading(battery_a2d);
    battery_sample_time = 0;
  }

  // Publish battery voltage every n ms
  if ((battery_publish_time += LOOP_DELAY_MS) > BATTERY_PUB_RATE_MS) {

    battery_a2d_avg = BatteryAvg.getAvg();
    // Y =  mx + b - b is roughly zener diode voltage drop (not perfectly linear)
    calculatedVoltage = 0;
    if (battery_a2d_avg > 20) {
      calculatedVoltage = (0.0071 * (float)battery_a2d_avg) + 9.80;
    }

    batteryMsg.data = calculatedVoltage;
    pub_battery.publish(&batteryMsg);
    battery_publish_time = 0;
  }

  nh.spinOnce();  // ROS Heartbeat / Communicate with ROS
  onboard_neopixel.setPixelColor(ONBOARD_NEOPIXEL_INDEX, 0, 63, 0);  // Set pixel to Green at end of loop
  onboard_neopixel.show();

  delay(LOOP_DELAY_MS);
}


////////////////////////////////////////////////////////////////////////////////
// UTILITIES
// Define the sensor outputs to receive
void setReports(void) {

  nh.loginfo("Body Arduino: Setting desired reports");
  if (myIMU.enableRotationVector() == true) {
    nh.loginfo("Body Arduino: Rotation vector enabled");
    nh.loginfo("Body Arduino: Output in form roll, pitch, yaw");
  } else {
    nh.loginfo("Body Arduino: Could not enable rotation vector");
  }
}

void send_speed_command(int device_id, int speed) {
  nh.loginfo("Body Arduino: Sending speed to motor");
  Wire.beginTransmission(device_id);  // transmit to device
  Wire.write((byte)speed);            // sends cmd
  Wire.endTransmission();             // stop transmitting
}
