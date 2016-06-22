#include <Arduino.h>

/*
Code for the Modern Device Motion Plug, which uses the MPU 9250 9-axis accelerometer/gyro/compass chip from Invensense.

Lightly modified version of AvrCopter.ino from rpicopter: https://github.com/rpicopter/ArduinoMotionSensorExample

Note: This is a 3.3v board. If you are using a 5 volt Arduino (or compatible),
plug the + line into 3.3v and use series resistors on SDA and SCL for voltage signal level shifting.
The standard value 10k resistors won't work with high speed I2C, but we found that 4.7k and 3.3k work fine.

Features:
  - uses FastWire and I2Cdev by Jeff Rowberg
  - DMP enabled
  - calculates and displays gyro and quaternions
  - This has been tested using Arduino 1.0.5, 1.6.1,
    a few variants of Arduino Pro Mini boards (atmega328), the Modern Device Robot Board Rev E. (atmega2560),
    Arduino Mega (atmega2560), the BBLeo (atmega32u4), and the JeeNode SMD (atmega328).

- This library specifies the MPU9250.
  If you have an MPU6050, MPU6500, or MPU9150,
  change #DEFINE MPU9250 to #DEFINE MPU6050,
  #DEFINE MPU6500, or #DEFINE MPU9150, respectively in inv_mpu.h.

- Default chip configuration values, from inv_mpu.h.
  Pulled out here for reference:
    test->reg_rate_div   = 0;    // 1kHz.
    test->reg_lpf        = 1;    // 188Hz.
    test->reg_gyro_fsr   = 0;    // 250dps.
    test->reg_accel_fsr  = 0x18; // 16g.
*/

#include <freeram.h>
#include <mpu.h>
#include <I2Cdev.h>

int ret;

// switch
const byte switchPin = 2;

// connect motor controller pins to Arduino digital pins
// switch
const byte resetSwitchPin = 2;
const byte startSwitchPin = 3;
// motor one
const byte enA = 10;
const byte in1 = 9;
const byte in2 = 8;
// motor two
const byte enB = 5;
const byte in3 = 7;
const byte in4 = 6;
// Speed difference between motors
float multiA = 1.0;
float multiB = 1.1;
int minSpeedA;
int minSpeedB;

float currentAngle = 0;          //Keep track of our current angle
float oldAngle = 0;              //Used to complete a smooth graph


void setup() {
  delay(2000); //Delay 2 seconds to settle the minitbot for MPU initialization
  Fastwire::setup(400,0);

    // set all the motor control pins to outputs
    pinMode(resetSwitchPin, INPUT);
    pinMode(startSwitchPin, INPUT);
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);

    Serial.begin(9600);
    ret = mympu_open(200);
    Serial.print("MPU init: "); Serial.println(ret);
    Serial.print("Free mem: "); Serial.println(freeRam());
    delay(100);

    //Set MaxSpeed based on slowest motor
    minSpeedA = 160;
    minSpeedB = 160;
}

unsigned int c = 0; //cumulative number of successful MPU/DMP reads
unsigned int np = 0; //cumulative number of MPU/DMP reads that brought no packet back
unsigned int err_c = 0; //cumulative number of MPU/DMP reads that brought corrupted packet
unsigned int err_o = 0; //cumulative number of MPU/DMP reads that had overflow bit set


void loop() {
    //errorReporting(); // turn on for debug information

   /* if (!(c%25)) { // output only every 25 MPU/DMP reads
	    Serial.print("Y: "); Serial.print(mympu.ypr[0]);
	    Serial.print(" P: "); Serial.print(mympu.ypr[1]);
	    Serial.print(" R: "); Serial.print(mympu.ypr[2]);
	    Serial.print("\tgy: "); Serial.print(mympu.gyro[0]);
	    Serial.print(" gp: "); Serial.print(mympu.gyro[1]);
	    Serial.print(" gr: "); Serial.println(mympu.gyro[2]);
	    delay(100);

    } */
    int startSensorVal = digitalRead(startSwitchPin);
    int resetSensorVal = digitalRead(resetSwitchPin);

     // Keep in mind the pullup means the pushbutton's
     // logic is inverted. It goes HIGH when it's open,
     // and LOW when it's pressed. Turn on pin 13 when the
     // button's pressed, and off when it's not:
     if (resetSensorVal == HIGH) {
         delay(200); // for debounce
         //gyroZeroVoltage = (analogRead(gyroPin) * gyroVoltage) / 1023;
     }
     if (startSensorVal == HIGH) {
        delay(200); // for debounce
        moveStraightRoutine (2000);
     }

}

void fullStop ()
{
  // now turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

}

void moveStraightRoutine (long runTime)
{
 int tAcceleration = 1000;
 int tMaxSpeed = runTime - tAcceleration;
 int minSpeedA = 140;
 int minSpeedB = 170;
 int maxSpeedA = 235;
 int maxSpeedB = 245;
 float dirSpeedA = 0;
 float dirSpeedB = 0;
 int cSpeedA = 0;
 int cSpeedB = 0;
 float addSpeedA = (maxSpeedA - minSpeedA)/tAcceleration;
 float addSpeedB = (maxSpeedB - minSpeedB)/tAcceleration;
 int K = 5;
 long startTime = millis();
 long cTime = 0;


 while (millis() - startTime < runTime) {
   //if (!(c%10)) { // output only every 10 MPU/DMP reads
   ret = mympu_update();
   currentAngle = mympu.ypr[0];
      //}
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);

  cTime = millis() - startTime;
  if (cTime < tAcceleration) {
      cSpeedA = minSpeedA + (cTime * addSpeedA);
      cSpeedB = minSpeedB + (cTime * addSpeedB);
  }
  else if (cTime < tMaxSpeed) {
      cSpeedA = maxSpeedA;
      cSpeedB = maxSpeedB;
  }
  else {
      cSpeedA = maxSpeedA - (cTime-tAcceleration-tMaxSpeed * addSpeedA);
      cSpeedB = maxSpeedB - (cTime-tAcceleration-tMaxSpeed * addSpeedB);
   }

 if (currentAngle >= 1)
 {
   dirSpeedA = sq(currentAngle)*K*-1;
   dirSpeedB = sq(currentAngle)*K;
 }
 else if (currentAngle <= -1)
 {
   dirSpeedA = sq(currentAngle)*K;
   dirSpeedB = sq(currentAngle)*K*-1;
 }
else if (currentAngle < 0)
 {
  dirSpeedA = currentAngle*K*-1;
  dirSpeedB = currentAngle*K;
 }
 else if (currentAngle > 0)
 {
   dirSpeedA = currentAngle*K;
   dirSpeedB = currentAngle*K*-1;
 }

 analogWrite (enA, constrain((cSpeedA + dirSpeedA), minSpeedA, 255));
 analogWrite (enB, constrain((cSpeedB + dirSpeedB), minSpeedB, 255));
 Serial.print (currentAngle);
 Serial.print (", A:");
 Serial.print min((cSpeedA + dirSpeedA),255);
 Serial.print (", B:");
 Serial.println min((cSpeedA + dirSpeedA),255);

   //DEBUG for Serial Monitor
   //Serial.println(currentAngle);

   // Debug for Cypress Chart Plotter, format - RX8 [h=43] @1Key1 @0Key1
   //Serial.print("C");
   //Serial.write((int)(currentAngle)>>8);
   //Serial.write((int)(oldAngle)&0xff);
   oldAngle = currentAngle;
  }
  void fullStop ();
}
