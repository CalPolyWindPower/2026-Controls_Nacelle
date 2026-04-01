#include <Arduino.h>
#include "AS5600.h"
#include <Wire.h>



AS5600 as5600;   //  use default Wire

const int SCL_firebeetle = 10;
const int SDA_firebeetle = 9;

const int t = 100;
const int dataset_size = 2;

float rpms[dataset_size + 1];



void setup() {

  Serial.begin(115200);

  Wire.begin(SDA_firebeetle,SCL_firebeetle);
  as5600.begin();  //  set direction pin.
  as5600.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
  int b = as5600.isConnected();
  Serial.print("Connect: ");
  Serial.println(b);
  delay(1000);

  // load rpm data into array to prevent error during main loop
  for (int i = 0; i<=dataset_size; i++) {
    rpms[i] = as5600.getAngularSpeed(AS5600_MODE_RPM);
    delay(t);
    Serial.println(String(i));
  }

}



void loop() {
  float rpm_average;

  // shifts every element in rpms[] to the right
  for (int i = dataset_size; i >= 1; i--) {
    rpms[i] = rpms[i-1];
  }
  rpms[0] = as5600.getAngularSpeed(AS5600_MODE_RPM); 

  // sums rpms[]
  for (int i = 0; i <= dataset_size; i++) {
    rpm_average += rpms[i];
  }

  rpm_average = rpm_average/dataset_size + 1;

  Serial.print("\tω = ");
  Serial.println(String(rpm_average));
  delay(100);

  // error checking; definitely room for a more in depth system
  int e = as5600.lastError();
  if (e != AS5600_OK){
    Serial.println(String(e));
  }

}
