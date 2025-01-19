#include "define.h"
#include "functions.h"

#include "MS5837.h"
#include <MPU6500_WE.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_INA219.h>
#include "CAN/Sense_can.h"
#include <Kalman.h>

MS5837 BAR30;
Adafruit_BMP280 bmp(&Wire2);
Adafruit_LIS3MDL MAG;
MPU6500_WE MPU = MPU6500_WE(&Wire2, MPU6500_ADDR);
Sense_can CAN;
Adafruit_INA219 INA;


bool PRINTLN_E = false, LD_E = false, BD_E = false, FD_E = false, BAR_E = false, BMP_E = false, MAG_E = false, MPU_E = false, INA_E = false;
bool RD_C = false, LD_C = false, BD_C = false, FD_C = false, BAR_C = false, BMP_C = false, MAG_C = false, MPU_C = false, INA_C = false;
bool disable_distance = true;

// BAR30 raw data
float ext_pressure = 0, ext_temp = 0, depth = 0, ext_altitude = 0;

// BMP raw data
float int_pressure = 0, int_temp = 0, int_altitude = 0;

// MPU raw data
double accelX = 9, accelY = 9, accelZ = 9;
double gyroX = 9, gyroY = 9, gyroZ = 9;

// MPU processed data
double roll = 9, pitch = 9;
Kalman kalmanX, kalmanY;
uint32_t timer = 0;
float kalPitch = 0, kalRoll = 0;

// MAG raw data
double magX = 9, magY = 9, magZ = 9;

// MAG processed data
double yaw = 9;

// Power monitoring data
double voltage = 9, current = 9;

// CAN data
bool enable_CAN = true;
float intPrTp[2] = {9, 9}; 				// PHT (NOT USED)
float yawDepth[2] = {9, 9};				// IMU_HI_DEPTH
float rollPitch[2] = {9, 9};			// IMU_LOW
float voltCurr[2] = {9, 9};				// Power monitoring

/**
 * Checks the 8pos DIP switch to see which sensors are on/off
 * True on boolean implies ON, False implies OFF
 * Pins are pulled to HIGH by default (implying ON)
 * When switch is turned ON, pin goes to GND/LOW (implying OFF)
 */
void checkSW() {
	PRINTLN_E = digitalRead(SW_PRINTLN);
	LD_E = digitalRead(SW_LD);
	BD_E = digitalRead(SW_BD);
	FD_E = digitalRead(SW_FD);
	BAR_E = digitalRead(SW_BAR);
	BMP_E = digitalRead(SW_BMP);
	MAG_E = digitalRead(SW_MAG);
	MPU_E = digitalRead(SW_MPU);

	INA_E = false;		// Special case, has no hardware switch
}

/**
 * Calls the setup functions of all the sensors
 * Checks the boolean values first before setting up
 */
void sensorStart() {
	if (INA_E) {
		inaSetup();
		if (INA_C) { Serial.println("INA setup done");}
	}
	if (BAR_E) {
		bar30Setup();
		if (BAR_C) { Serial.println("BAR30 setup done");}
	}
	if (BMP_E) {
		bmpSetup();
		if (BMP_C) { Serial.println("BMP280 setup done");}
	}
	if (MAG_E) {
		magSetup();
		if (MAG_C) { Serial.println("LIS3MDL setup done");}
	}
	if (MPU_E) {
		mpuSetup();
		if (MPU_C) { Serial.println("MPU6500 setup done");}
	}
}

/**
 * Helper function to check if all sensors are connected
 * Change disable_distance to TRUE if distance sensors are in use
 */
void sensorConn() {
	if (disable_distance) {
		if (BAR_E && BMP_E && MAG_E && MPU_E && INA_E) {
			Serial.println("All sensors (less 4x distance) connected");
			flash(4, 1, 1);
		}
		else if (BAR_E && BMP_E && MAG_E && MPU_E) {
			Serial.println("All sensors (less 4x distance and power) connected");
  		flash(2, 1, 1);
		}
	}
	else {
		if (BAR_E && BMP_E && MAG_E && MPU_E && INA_E && LD_E && BD_E && FD_E) {
			Serial.println("All sensors (incl. 4x distance) connected");
			flash(7, 1, 1);
		}
	}
}

/**
 * BAR30 setup process
 */
void bar30Setup() {
	BAR30.setModel(MS5837::MS5837_30BA);
	BAR30.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
	Serial.println("Starting");
	delay(20);
	int count = 0;
	while (!BAR30.init() && count < 10) {
		Serial.println("BAR30 init failed!");
		count += 1;
		delay(100);
	}
	if (count < 10) {
		Serial.println("BAR30 connected");
		BAR_C = true;
	}
	else {
		Serial.println("BAR30 NOT connected");
	}
}

/**
 * BMP280 setup process
 */
void bmpSetup() {
	int count = 0;
	while (!bmp.begin(0x76) && count < 10) {
		Serial.println("BMP280 init failed");
		count += 1;
		delay(100);
	}
	if (count < 10) {
		Serial.println("BMP280 connected");
		BMP_C = true;
	}
	else {
		Serial.println("BMP280 NOT connected");
	}
}

/**
 * LIS3MDL setup process
 */
void magSetup() {
	int count = 0;

	// Serial.println("Adafruit LIS3MDL test!");
	// Try to initialize!
	// while (! lis3mdl.begin_I2C() && count < 10) {          // hardware I2C mode, can pass in address & alt Wire
	// while (! lis3mdl.begin_SPI(LIS3MDL_CS) && count < 10) {  // hardware SPI mode
	while (!MAG.begin_SPI(CS_MAG, CLK, MISO, MOSI) && count < 10) { // soft SPI
		Serial.println("Failed to find LIS3MDL chip");
		count += 1;
		delay(100);
	}
	if (count < 10) {
		Serial.println("MAG connected");
		MAG_C = true;
	}
	else {
		Serial.println("MAG NOT connected");
	}

	if (MAG_C) {
		MAG.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
		Serial.print("Performance mode set to: ");
		switch (MAG.getPerformanceMode()) {
			case LIS3MDL_LOWPOWERMODE: Serial.println("Low"); break;
			case LIS3MDL_MEDIUMMODE: Serial.println("Medium"); break;
			case LIS3MDL_HIGHMODE: Serial.println("High"); break;
			case LIS3MDL_ULTRAHIGHMODE: Serial.println("Ultra-High"); break;
		}

		MAG.setOperationMode(LIS3MDL_CONTINUOUSMODE);
		Serial.print("Operation mode set to: ");
		// Single shot mode will complete conversion and go into power down
		switch (MAG.getOperationMode()) {
			case LIS3MDL_CONTINUOUSMODE: Serial.println("Continuous"); break;
			case LIS3MDL_SINGLEMODE: Serial.println("Single mode"); break;
			case LIS3MDL_POWERDOWNMODE: Serial.println("Power-down"); break;
		}

		MAG.setDataRate(LIS3MDL_DATARATE_155_HZ);
		// You can check the datarate by looking at the frequency of the DRDY pin
		Serial.print("Data rate set to: ");
		switch (MAG.getDataRate()) {
			case LIS3MDL_DATARATE_0_625_HZ: Serial.println("0.625 Hz"); break;
			case LIS3MDL_DATARATE_1_25_HZ: Serial.println("1.25 Hz"); break;
			case LIS3MDL_DATARATE_2_5_HZ: Serial.println("2.5 Hz"); break;
			case LIS3MDL_DATARATE_5_HZ: Serial.println("5 Hz"); break;
			case LIS3MDL_DATARATE_10_HZ: Serial.println("10 Hz"); break;
			case LIS3MDL_DATARATE_20_HZ: Serial.println("20 Hz"); break;
			case LIS3MDL_DATARATE_40_HZ: Serial.println("40 Hz"); break;
			case LIS3MDL_DATARATE_80_HZ: Serial.println("80 Hz"); break;
			case LIS3MDL_DATARATE_155_HZ: Serial.println("155 Hz"); break;
			case LIS3MDL_DATARATE_300_HZ: Serial.println("300 Hz"); break;
			case LIS3MDL_DATARATE_560_HZ: Serial.println("560 Hz"); break;
			case LIS3MDL_DATARATE_1000_HZ: Serial.println("1000 Hz"); break;
		}
		
		MAG.setRange(LIS3MDL_RANGE_4_GAUSS);
		Serial.print("Range set to: ");
		switch (MAG.getRange()) {
			case LIS3MDL_RANGE_4_GAUSS: Serial.println("+-4 gauss"); break;
			case LIS3MDL_RANGE_8_GAUSS: Serial.println("+-8 gauss"); break;
			case LIS3MDL_RANGE_12_GAUSS: Serial.println("+-12 gauss"); break;
			case LIS3MDL_RANGE_16_GAUSS: Serial.println("+-16 gauss"); break;
		}

		MAG.setIntThreshold(500);
		MAG.configInterrupt(false, false, true, // enable z axis
														true, // polarity
														false, // don't latch
														true); // enabled!
	}
}

/**
 * MPU6500 setup process
 */
void mpuSetup() {
	int count = 0;
	while(!MPU.init() && count < 10){
		Serial.println("MPU6500 does not respond");
		delay(100);
		count += 1;
	}
	if (count < 10) {
		Serial.println("MPU6500 is connected");
		MPU_C = true;
		MPU.enableGyrDLPF();
		MPU.setGyrDLPF(MPU6500_DLPF_6);
		MPU.setSampleRateDivider(5);
		MPU.setGyrRange(MPU6500_GYRO_RANGE_250);
		MPU.setAccRange(MPU6500_ACC_RANGE_2G);
		MPU.enableAccDLPF(true);
		MPU.setAccDLPF(MPU6500_DLPF_6);
		mpuCalib();
	}
	else if (count > 10) {
		Serial.println("MPU6500 is NOT connected");
		Serial.println("I died");
	}
}

/**
 * MPU6500 auto calibration process
 */
void mpuCalib() {
	Serial.println("Position you MPU6500 flat and don't move it - calibrating...");
	delay(1000);
	MPU.autoOffsets();
	Serial.println("Done!");
}

/**
 * CAN bus setup process
 */
void CANSetup() {
	CAN.Setup_can(9600, 500*1000); //set baudrate for serial monitoring and bitrate for can
}

/**
 * Function to get BAR30 readings
 */
void bar30Read() {
	BAR30.read();

	ext_pressure = BAR30.pressure();
	ext_temp = BAR30.temperature();
	depth = BAR30.depth();
	ext_altitude = BAR30.altitude();

	Serial.print("Pressure: ");
	Serial.print(ext_pressure); 
	Serial.println(" mbar");
	
	Serial.print("Temperature: "); 
	Serial.print(ext_temp); 
	Serial.println(" deg C");
	
	Serial.print("Depth: "); 
	Serial.print(depth); 
	Serial.println(" m");
	
	Serial.print("Altitude: "); 
	Serial.print(ext_altitude); 
	Serial.println(" m above mean sea level");

	yawDepth[1] = depth;
}

/**
 * Function to get BMP280 readings
 */
void bmpRead() {
	int_temp = bmp.readTemperature();
	int_pressure = bmp.readPressure() / 100;
	int_altitude = bmp.readAltitude(1013.25);
	Serial.println("\nBMP280 Reading:");
	Serial.print("Temperature: "); Serial.print(int_temp); Serial.println(" *C");
	Serial.print("Pressure: "); Serial.print(int_pressure); Serial.println(" mb");
	Serial.print("Altitude: ");	Serial.print(int_altitude);	Serial.println(" m");
	intPrTp[0] = int_pressure;
	intPrTp[1] = int_temp;
}

/**
 * Function to get LIS3MDL (MAG) readings
 */
void magRead() {
	MAG.read();      // get X Y and Z data at once
	// Then print out the raw data
	magX = MAG.x;
	magY = MAG.y;
	magZ = MAG.z;
	Serial.print("\nMAG Reading:\nX:  "); Serial.print(magX); Serial.print("\tY:  "); Serial.print(magY); Serial.print("\tZ:  "); Serial.println(magZ); 
	/* Or....get a new sensor event, normalized to uTesla */
	sensors_event_t event; 
	MAG.getEvent(&event);
	/* Display the results (magnetic field is measured in uTesla) */
	Serial.print("X: "); Serial.print(event.magnetic.x);
	Serial.print("\tY: "); Serial.print(event.magnetic.y); 
	Serial.print("\tZ: "); Serial.print(event.magnetic.z); 
	Serial.println(" uTesla ");
}

/**
 * Function to get MPU6500 readings
 */
void mpuRead() {
	xyzFloat gValue = MPU.getGValues();
	xyzFloat gyr = MPU.getGyrValues();
	float temp = MPU.getTemperature();
	float resultantG = MPU.getResultantG(gValue);

  accelX = gValue.x;
  accelY = gValue.y;
  accelZ = gValue.z;
  gyroX = gyr.x;
  gyroY = gyr.y;
  gyroZ = gyr.z;

	Serial.print("\nMPU Reading:\nAcceleration in g (x,y,z):\t");
	Serial.print(accelX); Serial.print("   "); Serial.print(accelY); Serial.print("   "); Serial.println(accelZ);
	Serial.print("Resultant g: "); Serial.println(resultantG);
  
  Serial.print("Gyroscope data in degrees/s:\t");
	Serial.print(gyroX); Serial.print("   "); Serial.print(gyroY); Serial.print("   "); Serial.println(gyroZ);

	Serial.print("Temperature in °C: ");
	Serial.println(temp);

	Serial.println("********************************************");
}

/**
 * Test function to call all 5 reads
 */
void allRead() {
	Serial.println();
	if (INA_C) { inaReadVoltage(); inaReadCurr(); delay(READ_DELAY);}
	if (BAR_C) { bar30Read(); delay(READ_DELAY);}			
	if (BMP_C) { bmpRead(); delay(READ_DELAY);}
	if (MAG_C) { magRead(); delay(READ_DELAY);}
	if (MPU_C) { mpuRead(); getRollPitch(); getRollPitchK(); getYaw(); delay(READ_DELAY);}
	if (enable_CAN) {
		Serial.println("-------------- CAN DATA HERE --------------");
		CAN.IMU_low(rollPitch, 0x13, 10);
		CAN.IMU_hi_depth(yawDepth, 0x14, 10);
		// CAN.PHT(intPrTp, 0x16, 10);
		CAN.VCS(voltCurr, 0x16, 10); 
		//Might be able to change to 0
	}
}

/**
 * Indicator LED off
 */
void turnOff() {
	digitalWrite(IND_1, LOW);
}

/**
 * Indicator LED flash
 * @param num: How many times the LED fast flashes
 * @param pin: Which LED to flash: 1: Blue, 2: Green
 * @param speed: How fast: 1: Fast, 2: Slow
 */
void flash(int num, int pin, int speed) {
	uint8_t led = 0;
	int time_delay = 0;
	switch(pin) {
		case 1:
			led = IND_1;
      break;
		case 2:
			led = IND_2;
      break;
	}
	switch(speed) {
		case 1:
			time_delay = FAST;
			break;
		case 2:
			time_delay = SLOW;
			break;
	}
	for (int i = 0; i < num; i +=1) {
		digitalWrite(led, HIGH);
		delay(time_delay);
		digitalWrite(led, LOW);
		delay(time_delay);
	}
}


/**
 * Setup the INA219 power monitoring board
 */
void inaSetup() {
	int count = 0;
	while (!INA.begin() && count < 10) {
		Serial.println("INA219 init failed!");
		count += 1;
		delay(100);
	}
	if (count < 10) {
		Serial.println("INA219 connected");
		INA_C = true;
	}
	else {
		Serial.println("INA219 NOT connected");
	}
}

/**
 * Read voltage value from INA219 power monitoring board
 */
void inaReadVoltage() {
	voltage = INA.getBusVoltage_V();
	Serial.print("Clean Battery Voltage:\t");
	Serial.println(voltage);
}

/**
 * Read current value from INA219 power monitoring board
 */
void inaReadCurr() {
  current = INA.getCurrent_mA();
  Serial.print("Clean Battery Current:\t");
  Serial.println(current);
}

/**
 * Read power value from INA219 power monitoring board
 */
void inaReadPwr() {
  float bus_pwr = INA.getPower_mW();
  Serial.print("Clean Battery Power:\t");
  Serial.println(bus_pwr);
}

/**
 * Software Reboot
 */
void reboot() {
  SCB_AIRCR = 0x05FA0004;
}

/**
 * Check software reset pin
 */
void check_reset() {
  if (digitalRead(RESET) == LOW) {
    reboot();
  }
}

/**
 * IMU data to roll/pitch
 */
void getRollPitch() {
  Serial.print("Raw AccelXYZ:\t");
  Serial.print(accelX); Serial.print("\t");
  Serial.print(accelY); Serial.print("\t");
  Serial.println(accelZ);
  Serial.print("Raw GyroXYZ:\t");
  Serial.print(gyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.println(gyroZ);
	
  roll = atan((double)accelY / hypotenuse((double)accelX, (double)accelZ)) * RAD_TO_DEG;
  pitch = atan2((double)-accelX, (double)accelZ) * RAD_TO_DEG;
  Serial.print("  Roll: ");
  Serial.print(roll);
  Serial.print("\t  Pitch: ");
  Serial.println(pitch);
  //rollPitch[0] = roll;
  //rollPitch[1] = pitch;
}

/**
 * Kalman Filter TKJ setup
 */
void kalmanSetup() {
	kalmanX.setAngle(roll);
	kalmanY.setAngle(pitch);
	timer = micros();
	kalmanX.setQbias(0.002f);
	kalmanY.setQbias(0.002f);	
}

/**
 * Kalman Filter TKJ
 */
void getRollPitchK() {
	double dt = (double)(micros() - timer) / 1000000;
	timer = micros();
	kalRoll = kalmanX.getAngle(roll, gyroX, dt);
	kalPitch = kalmanY.getAngle(pitch, gyroY, dt);
	Serial.print("K Roll: ");
	Serial.print(kalRoll);
	Serial.print("\tK Pitch: ");
	Serial.println(kalPitch);
	rollPitch[0] = kalRoll;
	rollPitch[1] = kalPitch;
}

/**
 * Get yaw values
 */
void getYaw() {
	double magdotaccel = (magX * accelX) + (magY * accelY) + (magZ * accelZ);
	double acceldotaccel = sqr(accelX) + sqr(accelY) + sqr(accelZ);
	double dotdiv = magdotaccel / acceldotaccel;
	double aX = accelX * dotdiv;
	double aY = accelY * dotdiv;
	double aZ = accelZ * dotdiv;
	double mX = magX - aX;
	double mY = magY - aY;
	double mZ = magZ - aZ;
	yaw = atan2(mX, mY) * 180 / M_PI;
	yawDepth[0] = yaw;
}

/* 
	Output the magnatometer values to the serial monitor for calibration purpose.
*/
void magCal_withGUI()
{
	MAG.read();// get X Y and Z data at once
	// Then print out the raw data
	magX = MAG.x;
	magY = MAG.y;
	magZ = MAG.z;

	sensors_event_t event; 
	MAG.getEvent(&event);
	
	Serial.print("Raw:0,0,0,0,0,0,");
	Serial.print(int(event.magnetic.x*10)); Serial.print(",");
	Serial.print(int(event.magnetic.y*10)); Serial.print(",");
	Serial.print(int(event.magnetic.z*10)); Serial.println("");

	// unified data
	Serial.print("Uni:0,0,0,0,0,0,");
	Serial.print(event.magnetic.x); Serial.print(",");
	Serial.print(event.magnetic.y); Serial.print(",");
	Serial.print(event.magnetic.z); Serial.println("");
}

// void magCal_withoutGUI()
// {
// 	MAG.read();// get X Y and Z data at once
// 	sensors_event_t event; 
// 	MAG.getEvent(&event);


	
// 	min_x = max_x = event.magnetic.x;
// 	min_y = max_y = event.magnetic.y;
// 	min_z = max_z = event.magnetic.z;
// 	delay(10);

// }