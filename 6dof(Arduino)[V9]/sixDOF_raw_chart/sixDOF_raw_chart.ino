#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

#include <Wire.h>

float angles[3]; // yaw pitch roll
int rawSixDof[6];
int data=5;

// Set the FreeSixIMU object
FreeSixIMU sixDOF = FreeSixIMU();

void setup() { 
  Serial.begin(9600);
  Wire.begin();
  
  delay(5);
  sixDOF.init(); //begin the IMU
  delay(5);
}

void loop() { 
  
   sixDOF.getRawValues(rawSixDof);
  // RX8 [h=43] @1Key1 @0Key1     
  Serial.print("C");     
  Serial.write(rawSixDof[0]>>8);     
  Serial.write(rawSixDof[0]&0xff);
  //Serial.print(rawSixDof[0]);
  //Serial.print(",");  
  //Serial.print(rawSixDof[1]);
  //Serial.print(" ");  
  //Serial.print(rawSixDof[3]);
  //Serial.print(",");  
  //Serial.println(rawSixDof[4]);

  delay(100); 
}

