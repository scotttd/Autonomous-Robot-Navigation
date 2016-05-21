#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

#include <Wire.h>

float angles[3]; // yaw pitch roll
int rawSixDof[6];

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
   
  Serial.print(rawSixDof[0]);
  Serial.print(",");  
  Serial.print(rawSixDof[1]);
  Serial.print(" ");  
  Serial.print(rawSixDof[2]);
  Serial.print(",");  
  Serial.println(rawSixDof[3]);
  Serial.print(",");  
  Serial.println(rawSixDof[4]);
   Serial.print(",");  
  Serial.println(rawSixDof[5]);


  delay(100); 
}

