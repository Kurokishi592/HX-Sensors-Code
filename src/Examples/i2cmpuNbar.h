#include <Arduino.h>
#include <Wire.h>
#include "MS5837.h"
#include <MPU6500_WE.h>

#define MPU6500_ADDR 0x68

MS5837 sensor;
MPU6500_WE myMPU6500 = MPU6500_WE(MPU6500_ADDR);

void setup() {
	Serial.begin(115200);
  Wire.begin();

	sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)

	Serial.println("Starting");
	delay(20);

	while (!sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(1000);
  }
	Serial.println("BAR30 connected");

  while(!myMPU6500.init()){
    Serial.println("MPU6500 does not respond");
  }
	Serial.println("MPU6500 is connected");
	myMPU6500.enableGyrDLPF();
	myMPU6500.setGyrDLPF(MPU6500_DLPF_6);
	myMPU6500.setSampleRateDivider(5);
	myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_250);
	myMPU6500.setAccRange(MPU6500_ACC_RANGE_2G);
	myMPU6500.enableAccDLPF(true);
	myMPU6500.setAccDLPF(MPU6500_DLPF_6);

	delay(200);
  
	Serial.println("Position you MPU6500 flat and don't move it - calibrating...");
  delay(1000);
  myMPU6500.autoOffsets();
  Serial.println("Done!");
}

void loop() {
	sensor.read();

  Serial.print("Pressure: "); 
  Serial.print(sensor.pressure()); 
  Serial.println(" mbar");
  
  Serial.print("Temperature: "); 
  Serial.print(sensor.temperature()); 
  Serial.println(" deg C");
  
  Serial.print("Depth: "); 
  Serial.print(sensor.depth()); 
  Serial.println(" m");
  
  Serial.print("Altitude: "); 
  Serial.print(sensor.altitude()); 
  Serial.println(" m above mean sea level");

	xyzFloat gValue = myMPU6500.getGValues();
  xyzFloat gyr = myMPU6500.getGyrValues();
  float temp = myMPU6500.getTemperature();
  float resultantG = myMPU6500.getResultantG(gValue);

  Serial.println("Acceleration in g (x,y,z):");
  Serial.print(gValue.x);
  Serial.print("   ");
  Serial.print(gValue.y);
  Serial.print("   ");
  Serial.println(gValue.z);
  Serial.print("Resultant g: ");
  Serial.println(resultantG);

  Serial.println("Gyroscope data in degrees/s: ");
  Serial.print(gyr.x);
  Serial.print("   ");
  Serial.print(gyr.y);
  Serial.print("   ");
  Serial.println(gyr.z);

  Serial.print("Temperature in °C: ");
  Serial.println(temp);

  Serial.println("********************************************");

	delay(500);
}