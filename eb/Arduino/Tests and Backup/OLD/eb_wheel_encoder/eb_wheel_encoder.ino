/* Wheel Encoder for ROS
   based upon Encoder Library - TwoKnobs Example from
   http://www.pjrc.com/teensy/td_libs_Encoder.html
   "This example code is in the public domain."

   NOTE: Attempted to add odomResetSubscriber, but found that Feather crashes any time the subscriber is added.
   So instead, counter is reset whenever ROS connects, using "if(!nh.connected() )..."
*/

#define USE_USBCON  // NEEDED FOR ATmega32u4 - FEATHER OR LEONARDO!!

#include <Encoder.h>
#include <ros.h>
#include <std_msgs/String.h>
//#include <std_msgs/Empty.h>
#include <wheel_control/WheelOdomRaw.h>
#include <wheel_control/WheelSpeedRaw.h>


const long ODOM_SAMPLE_TIME_INTERVAL = 50L; // 50ms - need to be fast to get frequent odom published
//const int SPEED_SAMPLE_DIVIDER = 2L; // 2x the Odom sample time - too short and it wont work for slow motor speeds

// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability

long wheelCountRight  = 0;
long wheelCountLeft  = 0;

ros::NodeHandle nh;
wheel_control::WheelOdomRaw odom_msg;
wheel_control::WheelSpeedRaw speed_msg;
ros::Publisher odom_pub("wheel_odom_raw", &odom_msg);
ros::Publisher speed_pub("wheel_speed_raw", &speed_msg);

/***
void reset_odom_callback(const std_msgs::Empty& cmd_msg) {
  nh.loginfo("Arduino Got RESET ODOM message");
  //wheelCountRight = 0;
  //wheelCountLeft = 0;
}
ros::Subscriber<std_msgs::Empty> odomResetSubscriber("/reset_odom", &reset_odom_callback);
***/

// Assign pins for quadrature encoder.
// MEGA pins with interrupt are: 2,3, plus 18,19,(serial1) 20,21 (I2C)
//Encoder wheelEncoderRight(2, 3);  // quadrature inputs for each motor
//Encoder wheelEncoderLeft(18, 19); // block use of Serial1!
// Feather pins with interrups are: 0,1,2,3
Encoder wheelEncoderLeft(1, 0);  // quadrature inputs for each motor
Encoder wheelEncoderRight(3, 2); //


long speedRight = 0;
long speedLeft = 0;
long lastSpeedCountRight = 0L;
long lastSpeedCountLeft = 0L;

unsigned long startTime = 0L;
unsigned long runTime = 0L;

unsigned long speedTime = 0L;
unsigned long lastSpeedTime = 0L;
long SpeedSampleDuration = 0L;

unsigned long loopEndTime = 0L;
boolean led_is_on = false;

///long speedLoopCount = 0L;


// For Speed Control (future feature)
//int targetSpeedRight = 0;
//int targetSpeedLeft = 0;
//int currentMotorCommandRight = 0;
//int currentMotorCommandLeft = 0;
//volatile boolean indexFound = false;
//volatile long indexPos = NOT_SET;


void setup() {
  //nh.getHardware()->setBaud(19200);
  nh.initNode();
  nh.advertise(odom_pub);
  nh.advertise(speed_pub);
  pinMode(LED_BUILTIN, OUTPUT);
  speedTime = millis();
  lastSpeedTime = speedTime;
  //nh.subscribe(odomResetSubscriber);
  
  delay(1); // assure time is not zero

}

void loop() {

if(!nh.connected() ) {
  // assume ROS boot/reset.  Reset counters
  while (!nh.connected() ){
    nh.spinOnce();
    // Do a fast blink
    if ( led_is_on ) {
      digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
      led_is_on = false;
    }
    else {
      digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
      led_is_on = true;
    }
    delay(50); 
  }
  wheelCountRight = 0;
  wheelCountLeft = 0;
  wheelEncoderRight.write(0);
  wheelEncoderLeft.write(0);
  
  nh.logwarn("Arduino RESET ODOM COUNTERS TO ZERO!");
}

  
  // this loop executes every ODOM_SAMPLE_TIME_INTERVAL
  loopEndTime =  millis() + ODOM_SAMPLE_TIME_INTERVAL;

  // Read Motor Encoders
  wheelCountLeft = wheelEncoderLeft.read();
  wheelCountRight = wheelEncoderRight.read();

  // calculate speed every so often (can't do too fast!)
  ///if (speedLoopCount++ >= (SPEED_SAMPLE_DIVIDER-1)) {
    ///speedLoopCount = 0;
    speedTime = millis();
    SpeedSampleDuration = speedTime - lastSpeedTime; // see exact time that passed for speed calculation
    lastSpeedTime = speedTime;

    // Speed in Ticks per Second:
    speedLeft = ((wheelCountLeft - lastSpeedCountLeft) * 1000L) / SpeedSampleDuration; // avoid floating point operation
    lastSpeedCountLeft = wheelCountLeft;
    speedRight = ((wheelCountRight - lastSpeedCountRight) * 1000L) / SpeedSampleDuration;
    lastSpeedCountRight = wheelCountRight;

    // Publish Speed in ticks/sec
    speed_msg.speed_ticks_right = speedRight;
    speed_msg.speed_ticks_left = speedLeft;
    speed_pub.publish( &speed_msg );

    // Toggle LED each time we calculate speed
    if ( led_is_on ) {
      digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
      led_is_on = false;
    }
    else {
      digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
      led_is_on = true;
    }
    //nh.spinOnce();

  ///}

  // Publish Odometry in ticks
  odom_msg.odom_ticks_right = wheelCountRight;
  odom_msg.odom_ticks_left  = wheelCountLeft;
  odom_pub.publish( &odom_msg );


  // keep loop to a tight time sync
  while ( millis() <= loopEndTime ) {
    nh.spinOnce();
    delay(2); // allow time for encoder ISRs to run?
  }


  /***
      // Speed Control
      if (0 == targetSpeedRight) {
        // STOP
        // sendMotorCommand(MOTOR_RIGHT, 0); TODO
      }
      else if (speedRight < (targetSpeedRight - SPEED_HYSTERYSIS)) {
        // need to go more positive (either faster forward or slower reverse)
        currentMotorCommandRight += 1; // TODO TUNE THIS
        // sendMotorCommand(MOTOR_RIGHT, currentMotorCommandRight);  TODO

      }
      else if (speedRight > (targetSpeedRight + SPEED_HYSTERYSIS)) {
        // need to go more negative (either slower forward or faster reverse)
        currentMotorCommandRight -= 1; // TODO TUNE THIS
        // sendMotorCommand(MOTOR_RIGHT, currentMotorCommandRight);  TODO

      }
    }


    /// EXAMPLE CODE:

    unsigned long LoopStartTime = millis();  // (at begining of loop)

    // keep loop to a tight 20ms
    int LoopTime = millis() - LoopStartTime;

    if( LoopTime > LOOP_TIME_MS )
    {
    // Enable this to test/assure that the loop completes in less than 20ms
    // Disabled for now becuase it reports a lot of 21/22 - need to fix this
    // PrintDebugDec("LoopTime = ", LoopTime);
    }
    else
    {
     while( LoopTime < (LOOP_TIME_MS - 5) )
     {
      // read sensors instead of sitting idle
      Read_Next_Analog_Port();    // read an analog port (need time between reads)
      // read the head sensor, so we don't miss short touches
      Status.HeadSensor |= ReadPCF8574(I2C_PCF8574_HEAD); // read capacitive sensor in head (just a touch sensor)
      delay(1);
      LoopTime = millis() - LoopStartTime; // see how much time is still left
     }
     delay( LOOP_TIME_MS - LoopTime );
    }




  ***/

}

