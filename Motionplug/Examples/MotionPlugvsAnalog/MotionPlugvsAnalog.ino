/* ------------- Motion Plug Demo Sketch -------------

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

#include "freeram.h"
#include "mpu.h"
#include "I2Cdev.h"

int ret;

/* Keep track of gyro angle over time
 * Connect Gyro to Analog Pin 0
 *
 * Sketch by eric barch / ericbarch.com
 * v. 0.1 - simple serial output
 *
 */

// switch
const byte switchPin = 2;
int gyroPin = 2;               //Gyro is connected to analog pin 0
float gyroVoltage = 5;         //Gyro is running at 5V
float gyroZeroVoltage = 2.44;   //Our Gyro happens to be zeroed at 2.44. Gyro default is normally zeroed at 2.5V
float gyroSensitivity = .007;  //Our example gyro is 7mV/deg/sec
float rotationThreshold = .25;   //Minimum deg/sec to keep track of - helps with gyro drifting

float currentAngle = 0;          //Keep track of our current angle
float oldAngle = 0;              //Used to complete a smooth graph


void setup() {
    Fastwire::setup(400,0);
    Serial.begin(9600);
    ret = mympu_open(200);
    Serial.print("MPU init: "); Serial.println(ret);
    Serial.print("Free mem: "); Serial.println(freeRam());
    
    pinMode(switchPin, INPUT);
    delay(100);	

}

unsigned int c = 0; //cumulative number of successful MPU/DMP reads
unsigned int np = 0; //cumulative number of MPU/DMP reads that brought no packet back
unsigned int err_c = 0; //cumulative number of MPU/DMP reads that brought corrupted packet
unsigned int err_o = 0; //cumulative number of MPU/DMP reads that had overflow bit set

void loop() {
  //This line converts the 0-1023 signal to 0-5V
  float gyroRate = (analogRead(gyroPin) * gyroVoltage) / 1023;

  //This line finds the voltage offset from sitting still
  gyroRate -= gyroZeroVoltage;

  //This line divides the voltage we found by the gyro's sensitivity
  gyroRate /= gyroSensitivity;

  //Ignore the gyro if our angular velocity does not meet our threshold
  if (gyroRate >= rotationThreshold || gyroRate <= -rotationThreshold) {
    //This line divides the value by 100 since we are running in a 10ms loop (1000ms/10ms)
    gyroRate /= 100;
    currentAngle += gyroRate;
  }

  //Keep our angle between 0-359 degrees
  if (currentAngle < 0)
    currentAngle += 360;
  else if (currentAngle > 359)
    currentAngle -= 360;


  
    ret = mympu_update();
    //errorReporting(); // turn on for debug information

   if (!(c%25)) { // output only every 25 MPU/DMP reads
	    Serial.print("Y: "); Serial.print(mympu.ypr[0]);
      Serial.print(" YA: ");Serial.print((int)(currentAngle));
	    Serial.print(" P: "); Serial.print(mympu.ypr[1]);
	    Serial.print(" R: "); Serial.print(mympu.ypr[2]);
	    Serial.print("\tgy: "); Serial.print(mympu.gyro[0]);
	    Serial.print(" gp: "); Serial.print(mympu.gyro[1]);
	    Serial.print(" gr: "); Serial.println(mympu.gyro[2]);
	    delay(100);	
  int sensorVal = digitalRead(switchPin);
  if (sensorVal == HIGH) {
     gyroZeroVoltage = (analogRead(gyroPin) * gyroVoltage) / 1023;
  }

    }
}

void errorReporting(){
  if (ret != 0){
      switch (ret) {
	case 0: c++; break;
	case 1: np++; return;
	case 2: err_o++; return;
	case 3: err_c++; return; 
	default: 
		Serial.print("READ ERROR!  ");
		Serial.println(ret);
      }
      Serial.print(np);
      Serial.print("  ");
      Serial.print(err_c);
      Serial.print(" ");
      Serial.print(err_o);
      Serial.print(" ");
  }
}
