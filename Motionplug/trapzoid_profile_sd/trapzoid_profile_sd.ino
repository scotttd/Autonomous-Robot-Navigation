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

//MPU Status
unsigned int c = 0; //cumulative number of successful MPU/DMP reads
unsigned int np = 0; //cumulative number of MPU/DMP reads that brought no packet back
unsigned int err_c = 0; //cumulative number of MPU/DMP reads that brought corrupted packet
unsigned int err_o = 0; //cumulative number of MPU/DMP reads that had overflow bit set


// connect motor controller pins to Arduino digital pins
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
  Fastwire::setup(400,0);

    // set all the motor control pins to outputs
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);

    Serial.begin(9600);
    ret = mympu_open(200);
    //Serial.print("MPU init: "); Serial.println(ret);
    //Serial.print("Free mem: "); Serial.println(freeRam());
    delay(2000); //Delay 2 seconds to settle the minitbot for MPU initialization
    moveStraightRoutine(5000);
}


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
 int tAcceleration = 1500;
 int tMaxSpeed = runTime - tAcceleration;
 int minSpeed = 140;
 int maxSpeed = 235;
 float dirSpeedA = 0;
 float dirSpeedB = 0;
 float addSpeed = ((double) maxSpeed - (double) minSpeed)/(double) tAcceleration;
 delay(10);
 Serial.print(addSpeed);
 int K = 5;
 long startTime = millis();
 long cTime = 0;
 int cSpeed = 0;

 digitalWrite(in1, HIGH);
 digitalWrite(in2, LOW);
 digitalWrite(in3, HIGH);
 digitalWrite(in4, LOW);

 while (millis() - startTime < runTime) {
   ret = mympu_update();
   currentAngle = mympu.ypr[0];

   cTime = millis() - startTime;
    if (cTime < tAcceleration) {
        cSpeed = minSpeed + (cTime * addSpeed);
    }
    else if (cTime < tMaxSpeed) {
        cSpeed = maxSpeed;
    }
    else {
        cSpeed = maxSpeed - ((cTime - tMaxSpeed)*addSpeed);
     }

/*
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
*/
 //Serial.print ("Speed: ");
 Serial.println (constrain(cSpeed, minSpeed, maxSpeed));
 //Serial.println (cSpeed);

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
