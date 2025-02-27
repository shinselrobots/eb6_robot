/* Wheel motor control for EB
   Quad Encoder Library (and TwoKnobs example) from  http://www.pjrc.com/teensy/td_libs_Encoder.html
*/
#include <Encoder.h>
#include <Wire.h>
#include <Servo.h>  // Wheel motor controled as a servo

#define USE_USBCON    // NEEDED FOR ATmega32u4 - FEATHER OR LEONARDO!!
#define MOTOR_PIN 11  // PWM port (one of many on the Feather32u)
#define DEBUG_MESSAGES 0

#define SPEED_CONTROL_RATE_MS 100
#define SPEED_DEBUG_PUB_RATE_MS 1000

const long LOOP_RATE_MS = 100;  // Max rate that motor controller will accept updates!
char buffer[40];

Encoder wheelEncoder(0, 1);  // quadrature inputs RX, TX, both are interrupt
// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability

long odomTicks = 0;  // +/- Distance traveled in ticks
long speedTicks = 0;
long lastSpeedCount = 0L;
long dbg_last_wheel_count = 0L;
int speed_debug_publish_time = 0;

unsigned long speedTime = 0L;
unsigned long lastSpeedTime = 0L;
long SpeedSampleDuration = 0L;
unsigned long loopEndTime = 0L;
boolean led_is_on = false;

Servo Servo;             // Servo object to control motor
int Command = 0;         // Command from computer, usually just a motor speed (+/- 100 percent)
int SpeedRequested = 0;  // Percent speed (-100 to +100) requested from computer
int speed_control_time = 0;


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  speedTime = millis();
  lastSpeedTime = speedTime;
  wheelEncoder.write(0);
  delay(1);             // assure time is not zero
  Serial.begin(57600);  // 115200);
  Serial.println("Wheel motor control starting...");

  // I2C for communicating with master controller
  Wire.begin(9);  // join i2c bus with address
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);

  // Initialize Motor control
  Servo.attach(MOTOR_PIN);  // Servo Pin
  Servo.write(90);          // 90 degrees = servo neutral / motor off

  delay(500);
  SpeedRequested = 0;
  controlMotors();
  delay(2000);

}

void loop() {
  // this loop executes every LOOP_RATE_MS
  loopEndTime = millis() + LOOP_RATE_MS;

  // Read Motor Encoders
  odomTicks = wheelEncoder.read();

  if (odomTicks != dbg_last_wheel_count) {
    // Serial.print(" Ticks = ");
    //Serial.println(odomTicks);
    dbg_last_wheel_count = odomTicks;
  }

  // calculate speed
  speedTime = millis();
  SpeedSampleDuration = speedTime - lastSpeedTime;  // see exact time that passed for speed calculation
  lastSpeedTime = speedTime;

  // Speed in Ticks per Second:
  speedTicks = ((odomTicks - lastSpeedCount) * 1000L) / SpeedSampleDuration;
  lastSpeedCount = odomTicks;


  if ((speed_debug_publish_time += SPEED_CONTROL_RATE_MS) > SPEED_DEBUG_PUB_RATE_MS) {

    if (speedTicks != 0) {
      Serial.print("SpeedRequested:  ");
      Serial.print(SpeedRequested);
      Serial.print("     speedTicks:  ");
      Serial.print(speedTicks);
      Serial.print("     Odom Ticks:  ");
      Serial.print(odomTicks);
      Serial.print("     Sample duration:  ");
      Serial.println(SpeedSampleDuration);
    }
    speed_debug_publish_time = 0;
  }

  // Every so often, update speed control
  //if ((speed_control_time += LOOP_RATE_MS) > SPEED_CONTROL_RATE_MS) {
  //SpeedRequested = 30;
  controlMotors();
  speed_control_time = 0;
  //}

  //Serial.println("Blink...");
  // Toggle LED each time we calculate speed
  if (led_is_on) {
    digitalWrite(LED_BUILTIN, LOW);  // turn the LED off by making the voltage LOW
    led_is_on = false;
  } else {
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
    led_is_on = true;
  }

  // keep loop to a tight time sync

  // calculate speed
  speedTime = millis();
  SpeedSampleDuration = speedTime - lastSpeedTime;  // see exact time that passed for speed calculation
  lastSpeedTime = speedTime;



  while (millis() <= loopEndTime) {
    //nh.spinOnce();
    delay(2);  // allow time for encoder ISRs to run?
  }
}

// Receive commands from controller via I2C
void receiveEvent(int bytes) {
  //Serial.println("RX");
  byte received_cmd = Wire.read();  // read one character from the I2C
  int system_cmd = (int8_t)received_cmd;

  if (system_cmd > 100) {
    Serial.print("Special Command Received! Command = ");
    Serial.println(system_cmd);
  } else {
  //TODO   Serial.print("Received Speed Command: ");
  //TODO   Serial.println(system_cmd);
    SpeedRequested = system_cmd;  // Set new motor speed
  }
}

// Return odom reading when requested by controller via I2C
void requestEvent() {
  char bufferDist[8];

  // TEMP FOR MEASURING SPEED
  //long dbg_speed = ((long)SpeedRequested * 10000l) + speedTicks;
  //ltoa(dbg_speed, bufferDist, 10);

  ltoa(odomTicks, bufferDist, 10);
  Serial.print("Sending: ");
  Serial.println(odomTicks);
  //Serial.print("chars: ");
  //Serial.println(bufferDist);
  Wire.write(bufferDist);  // respond with bytes
}

void sendMotorCmd(int motor_speed_percent) {
  // Convert from +/- 0-100 percent to servo degrees expected by servo library
  // map(inputValue, fromLow, fromHigh, toLow, toHigh)

  int ServoCmd = 90;
  if ((motor_speed_percent > -5) && (motor_speed_percent < -5)) {
    ServoCmd = 90;
  } else {
    int constrained_speed = constrain(motor_speed_percent, -100, 100);  // +/- 100 percent
    ServoCmd = map(constrained_speed, -100, 100, 20, 160);              // map to 20 --> 160 degrees
  }
  //Serial.print("Sending servo Degrees: ");
  //Serial.println(ServoCmd);
  Servo.write(ServoCmd);  // send the command
}

////////////////////////////////////////////////////////////////////////////////
// Speed Control

void controlMotors() {

  // A basic active speed control.  Ramps motor speed to desired target, and adjusts for load changes.
  // Calculate current vs. requested speed, and control motors for constant speed
  // This gets called once each time a velocity update arrives (about every ???ms)
  // NOTE:  all calculations are done in range -1.0 to 1.0, representing % of max motor speed

  int SpeedCmd = SpeedRequested;
  const int max_ticks = 0;

  const double RAMP_INCREMENT = 0.001;                   //0.02
  const double HYSTERYSIS = 0.02;                        // 0.04
  const double FAST_RAMP_HYSTERYSIS = HYSTERYSIS * 8.0;  // 4.0
  const double MAX_RAMP = 0.4;                           // 0.2

  const double TURN_INCREMENT = 0.01;                         // 0.02
  const double TURN_HYSTERYSIS = 0.02;                        // 0.05
  const double FAST_TURN_HYSTERYSIS = TURN_HYSTERYSIS * 4.0;  // 4.0
  const double MAX_TURN_COMPENSATION = 0.3;                   // 0.2

  const double MAX_SPEED_METERS_PER_SECOND = 3.0;  // fastest speed robot is capable of moving
  const double MAX_TURN_RADIANS_PER_SECOND = 8.0;  // (Sabertooth speed = 1.0)

  double speedCmd = 0.0;
  double turnCmd = 0.0;

  // Calculate feedback from Odom, scaled as percent of max motor speed
  double feedback = 0.0;
  if (0.0 != speedTicks)  // prevent creep due to "+100" below
  {
    feedback = (double)(speedTicks + 100) / 2000.0;
  }


  // TODO - MOVE THIS INTO SPEED CONTROL
  // Handle wheel speed command changes
  // TODO: if needed, add slow ramp here
  sendMotorCmd(SpeedCmd);  // Send command to motor
}
