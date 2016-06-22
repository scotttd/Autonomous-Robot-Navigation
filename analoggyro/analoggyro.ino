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
float gyroZeroVoltage = 2.47;   //Our Gyro happens to be zeroed at 2.44. Gyro default is normally zeroed at 2.5V
float gyroSensitivity = .007;  //Our example gyro is 7mV/deg/sec
float rotationThreshold = .25;   //Minimum deg/sec to keep track of - helps with gyro drifting

float currentAngle = 0;          //Keep track of our current angle
float oldAngle = 0;              //Used to complete a smooth graph

void setup() {
  Serial.begin (9600);
  pinMode(switchPin, INPUT);

}

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

  //DEBUG for Serial Monitor
  //Serial.println(currentAngle);
  
  // Debug for Cypress Chart Plotter, format - RX8 [h=43] @1Key1 @0Key1     
  Serial.print("C");     
  Serial.write((int)(oldAngle)>>8);     
  Serial.write((int)(currentAngle)&0xff);
  oldAngle = currentAngle;
  delay(10);
  int sensorVal = digitalRead(switchPin);
  if (sensorVal == HIGH) {
     gyroZeroVoltage = (analogRead(gyroPin) * gyroVoltage) / 1023;
  }


}
