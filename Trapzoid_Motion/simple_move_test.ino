#include <Arduino.h>

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

void setup() {
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

    //Set MaxSpeed based on slowest motor
    minSpeedA = 160;
    minSpeedB = 160;
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
        moveStraightRoutine (5000);
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
 float addSpeedA = (maxSpeedA - minSpeedA)/tAcceleration;
 float addSpeedB = (maxSpeedB - minSpeedB)/tAcceleration;
 int K = 5;
 long startTime = millis();

 //set motor directions
 digitalWrite(in1, HIGH);
 digitalWrite(in2, LOW);
 digitalWrite(in3, HIGH);
 digitalWrite(in4, LOW);

  for (long tI = 0; tI < tAcceleration; tI = millis()-startTime) {
  analogWrite (enA, minSpeedA + (tI * addSpeedA));
  analogWrite (enB, minSpeedB + (tI * addSpeedB));
  }
  while (millis() - startTime < tMaxSpeed) {
  analogWrite (enA, maxSpeedA);
  analogWrite (enB, maxSpeedB);
  }
  for (long tI = 0; millis() - startTime < runTime; tI = millis()-startTime-tAcceleration-tMaxSpeed) {
  analogWrite (enA, maxSpeedA - (tI * addSpeedA));
  analogWrite (enB, maxSpeedB - (tI * addSpeedA));
  }
   fullStop ();
  }
