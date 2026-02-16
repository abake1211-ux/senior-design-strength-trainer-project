#include <LSM6DS3.h>
#include <Wire.h>

//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A
float aX, aY, aZ, gX, gY, gZ, a_mag, g_mag;
const unsigned long SAMPLE_PERIOD_US = 5000;  // 5 ms = 200 Hz
unsigned long lastSampleTime = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial);

  myIMU.settings.accelSampleRate = 208;
  myIMU.settings.gyroSampleRate  = 208;

  //Call .begin() to configure the IMUs
  if (myIMU.begin() != 0) {
    Serial.println("Device error");
  } else {
    Serial.println("aX,aY,aZ,gX,gY,gZ");
  }
}

void loop() {
  unsigned long now = micros();

  if (now - lastSampleTime >= SAMPLE_PERIOD_US) {
  lastSampleTime += SAMPLE_PERIOD_US;

  // read the acceleration data
  aX = myIMU.readFloatAccelX();
  aY = myIMU.readFloatAccelY();
  aZ = myIMU.readFloatAccelZ();
  gX = myIMU.readFloatGyroX();
  gY = myIMU.readFloatGyroY();
  gZ = myIMU.readFloatGyroZ();

  a_mag = sqrt(aX*aX + aY*aY + aZ*aZ);
  g_mag = sqrt(gX*gX + gY*gY + gZ*gZ);

    // print the data in CSV format
    Serial.print(a_mag, 3);
    Serial.print(',');
    Serial.print(g_mag, 3);
    Serial.println();
  }
}