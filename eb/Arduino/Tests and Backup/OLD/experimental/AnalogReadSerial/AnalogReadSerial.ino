#include <SAMD_AnalogCorrection.h>

/*
  AnalogReadSerial

  Reads an analog input on pin 0, prints the result to the Serial Monitor.
  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/AnalogReadSerial
*/

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {

  int sensorValue = 0;
      
  int sensorValue0 = analogRead(A0);
  Serial.print("A0 = ");
  Serial.println(sensorValue0);
  delay(10);        // delay in between reads for stability
      
  int sensorValue1 = analogRead(A1);
  Serial.print("A1 = ");
  Serial.println(sensorValue1);
  delay(10);        // delay in between reads for stability
/*      
  int sensorValue2 = analogRead(A2);
  Serial.print("A2 = ");
  Serial.println(sensorValue2);
  delay(10);        // delay in between reads for stability
      
  int sensorValue3 = analogRead(A3);
  Serial.print("A3 = ");
  Serial.println(sensorValue3);
  delay(10);        // delay in between reads for stability
      
  int sensorValue4 = analogRead(A4);
  Serial.print("A4 = ");
  Serial.println(sensorValue4);
  delay(10);        // delay in between reads for stability
      
  int sensorValue5 = analogRead(A5);
  Serial.print("A5 = ");
  Serial.println(sensorValue5);
  delay(10);        // delay in between reads for stability
      
  int sensorValue6 = analogRead(A6);
  Serial.print("A6 = ");
  Serial.println(sensorValue6);
  delay(10);        // delay in between reads for stability
*/      
  Serial.println("-----------");
  delay(500);        // delay in between reads for stability
 
}





