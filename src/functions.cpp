#include "define.h"
#include "functions.h"

#include "MS5837.h"
#include <MPU6500_WE.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_INA219.h>
#include "CAN/Sense_new_can.h"
#include <Kalman.h>
#include <SD.h>
#include <SPI.h>
#include <string>

// For disabling prints when necessary
#define PRINT(x) if (PRINTLN_E) Serial.print(x)
#define PRINTLN(x) if (PRINTLN_E) Serial.println(x)

// For disabling SD logging when necessary
#define LOGLN(x) if (SDLOG_E) logToSD(x)

MS5837 BAR30;
Adafruit_BMP280 bmp(&Wire2);
Adafruit_LIS3MDL MAG;
MPU6500_WE MPU = MPU6500_WE(&Wire2, MPU6500_ADDR);
Sense_can CAN;
Adafruit_INA219 INA;

// FOR SD Card
const int chipSelect = BUILTIN_SDCARD;

bool PRINTLN_E = false, SDLOG_E = false, BD_E = false, FD_E = false, BAR_E = false, BMP_E = false, MAG_E = false, MPU_E = false, INA_E = false;
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
float intPrTp[2] = {9, 9};	 // PHT (NOT USED)
float yawDepth[2] = {9, 9};	 // IMU_HI_DEPTH
float rollPitch[2] = {9, 9}; // IMU_LOW
float voltCurr[2] = {9, 9};	 // Power monitoring

/**
 * Checks the 8pos DIP switch to see which sensors are on/off
 * True on boolean implies ON, False implies OFF
 * Pins are pulled to HIGH by default (implying ON)
 * When switch is turned ON, pin goes to GND/LOW (implying OFF)
 */
void checkSW()
{
	// RD_E = digitalRead(SW_RD);
	PRINTLN_E = digitalRead(SW_PRINTLN);
	SDLOG_E = digitalRead(SW_SDLOG);
	// BD_E = digitalRead(SW_BD); Wait for the broken switch to be fixed
	FD_E = digitalRead(SW_FD);
	BAR_E = digitalRead(SW_BAR);
	BMP_E = digitalRead(SW_BMP);
	MAG_E = digitalRead(SW_MAG);
	MPU_E = digitalRead(SW_MPU);

	INA_E = false; // Special case, has no hardware switch
}

/**
 * Calls the setup functions of all the sensors
 * Checks the boolean values first before setting up
 */
void sensorStart()
{
	if (PRINTLN_E)
	{
		Serial.println("Serial is enabled");
	}
	if (SDLOG_E)
	{
		Serial.println("Logging is enabled");
	}
	if (INA_E)
	{
		inaSetup();
		if (INA_C)
		{
			PRINTLN("INA setup done");
		}
	}
	if (BAR_E)
	{
		bar30Setup();
		if (BAR_C)
		{
			PRINTLN("BAR30 setup done");
		}
	}
	if (BMP_E)
	{
		bmpSetup();
		if (BMP_C)
		{
			PRINTLN("BMP280 setup done");
		}
	}
	if (MAG_E)
	{
		magSetup();
		if (MAG_C)
		{
			PRINTLN("LIS3MDL setup done");
		}
	}
	if (MPU_E)
	{
		mpuSetup();
		if (MPU_C)
		{
			PRINTLN("MPU6500 setup done");
		}
	}
}

/**
 * Helper function to check if all sensors are connected
 * Change disable_distance to TRUE if distance sensors are in use
 */
void sensorConn()
{
	if (disable_distance)
	{
		if (BAR_E && BMP_E && MAG_E && MPU_E && INA_E)
		{
			PRINTLN("All sensors (less 4x distance) connected");
			flash(4, 1, 1);
		}
		else if (BAR_E && BMP_E && MAG_E && MPU_E)
		{
			PRINTLN("All sensors (less 4x distance and power) connected");
			flash(2, 1, 1);
		}
	}
	else
	{
		if (BAR_E && BMP_E && MAG_E && MPU_E && INA_E && BD_E && FD_E)
		{
			PRINTLN("All sensors (incl. 2x distance) connected");
			flash(7, 1, 1);
		}
	}
}

/**
 * BAR30 setup process
 */
void bar30Setup()
{
	BAR30.setModel(MS5837::MS5837_30BA);
	BAR30.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
	PRINTLN("Starting");
	delay(20);
	int count = 0;
	while (!BAR30.init() && count < 10)
	{
		PRINTLN("BAR30 init failed!");
		count += 1;
		delay(100);
	}
	if (count < 10)
	{
		PRINTLN("BAR30 connected");
		BAR_C = true;
	}
	else
	{
		PRINTLN("BAR30 NOT connected");
	}
}

/**
 * BMP280 setup process
 */
void bmpSetup()
{
	int count = 0;
	while (!bmp.begin(0x76) && count < 10)
	{
		PRINTLN("BMP280 init failed");
		count += 1;
		delay(100);
	}
	if (count < 10)
	{
		PRINTLN("BMP280 connected");
		BMP_C = true;
	}
	else
	{
		PRINTLN("BMP280 NOT connected");
	}
}

/**
 * LIS3MDL setup process
 */
void magSetup()
{
	int count = 0;

	// PRINTLN("Adafruit LIS3MDL test!");
	// Try to initialize!
	// while (! lis3mdl.begin_I2C() && count < 10) {          // hardware I2C mode, can pass in address & alt Wire
	// while (! lis3mdl.begin_SPI(LIS3MDL_CS) && count < 10) {  // hardware SPI mode
	while (!MAG.begin_SPI(CS_MAG, CLK, MISO, MOSI) && count < 10)
	{ // soft SPI
		PRINTLN("Failed to find LIS3MDL chip");
		count += 1;
		delay(100);
	}
	if (count < 10)
	{
		PRINTLN("MAG connected");
		MAG_C = true;
	}
	else
	{
		PRINTLN("MAG NOT connected");
	}

	if (MAG_C)
	{
		MAG.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
		PRINT("Performance mode set to: ");
		switch (MAG.getPerformanceMode())
		{
		case LIS3MDL_LOWPOWERMODE:
			PRINTLN("Low");
			break;
		case LIS3MDL_MEDIUMMODE:
			PRINTLN("Medium");
			break;
		case LIS3MDL_HIGHMODE:
			PRINTLN("High");
			break;
		case LIS3MDL_ULTRAHIGHMODE:
			PRINTLN("Ultra-High");
			break;
		}

		MAG.setOperationMode(LIS3MDL_CONTINUOUSMODE);
		PRINT("Operation mode set to: ");
		// Single shot mode will complete conversion and go into power down
		switch (MAG.getOperationMode())
		{
		case LIS3MDL_CONTINUOUSMODE:
			PRINTLN("Continuous");
			break;
		case LIS3MDL_SINGLEMODE:
			PRINTLN("Single mode");
			break;
		case LIS3MDL_POWERDOWNMODE:
			PRINTLN("Power-down");
			break;
		}

		MAG.setDataRate(LIS3MDL_DATARATE_155_HZ);
		// You can check the datarate by looking at the frequency of the DRDY pin
		PRINT("Data rate set to: ");
		switch (MAG.getDataRate())
		{
		case LIS3MDL_DATARATE_0_625_HZ:
			PRINTLN("0.625 Hz");
			break;
		case LIS3MDL_DATARATE_1_25_HZ:
			PRINTLN("1.25 Hz");
			break;
		case LIS3MDL_DATARATE_2_5_HZ:
			PRINTLN("2.5 Hz");
			break;
		case LIS3MDL_DATARATE_5_HZ:
			PRINTLN("5 Hz");
			break;
		case LIS3MDL_DATARATE_10_HZ:
			PRINTLN("10 Hz");
			break;
		case LIS3MDL_DATARATE_20_HZ:
			PRINTLN("20 Hz");
			break;
		case LIS3MDL_DATARATE_40_HZ:
			PRINTLN("40 Hz");
			break;
		case LIS3MDL_DATARATE_80_HZ:
			PRINTLN("80 Hz");
			break;
		case LIS3MDL_DATARATE_155_HZ:
			PRINTLN("155 Hz");
			break;
		case LIS3MDL_DATARATE_300_HZ:
			PRINTLN("300 Hz");
			break;
		case LIS3MDL_DATARATE_560_HZ:
			PRINTLN("560 Hz");
			break;
		case LIS3MDL_DATARATE_1000_HZ:
			PRINTLN("1000 Hz");
			break;
		}

		MAG.setRange(LIS3MDL_RANGE_4_GAUSS);
		PRINT("Range set to: ");
		switch (MAG.getRange())
		{
		case LIS3MDL_RANGE_4_GAUSS:
			PRINTLN("+-4 gauss");
			break;
		case LIS3MDL_RANGE_8_GAUSS:
			PRINTLN("+-8 gauss");
			break;
		case LIS3MDL_RANGE_12_GAUSS:
			PRINTLN("+-12 gauss");
			break;
		case LIS3MDL_RANGE_16_GAUSS:
			PRINTLN("+-16 gauss");
			break;
		}

		MAG.setIntThreshold(500);
		MAG.configInterrupt(false, false, true, // enable z axis
							true,				// polarity
							false,				// don't latch
							true);				// enabled!
	}
}

/**
 * MPU6500 setup process
 */
void mpuSetup()
{
	int count = 0;
	while (!MPU.init() && count < 10)
	{
		PRINTLN("MPU6500 does not respond");
		delay(100);
		count += 1;
	}
	if (count < 10)
	{
		PRINTLN("MPU6500 is connected");
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
	else if (count > 10)
	{
		PRINTLN("MPU6500 is NOT connected");
		PRINTLN("I died");
	}
}

/**
 * MPU6500 auto calibration process
 */
void mpuCalib()
{
	PRINTLN("Position you MPU6500 flat and don't move it - calibrating...");
	delay(1000);
	MPU.autoOffsets();
	PRINTLN("Done!");
}

/**
 * CAN bus setup process
 */
void CANSetup()
{
	// CAN.Setup_can(9600, 500*1000); //set baudrate for serial monitoring and bitrate for can
	CAN.Setup_can(500 * 1000);
}

/**
 * Function to get BAR30 readings
 */
void bar30Read()
{
	BAR30.read();

	ext_pressure = BAR30.pressure();
	ext_temp = BAR30.temperature();
	depth = BAR30.depth();
	ext_altitude = BAR30.altitude();

	// PRINT("Pressure: ");
	// PRINT(ext_pressure);
	// PRINTLN(" mbar");

	// PRINT("Temperature: ");
	// PRINT(ext_temp);
	// PRINTLN(" deg C");

	// PRINT("Depth: ");
	// PRINT(depth);
	// PRINTLN(" m");

	// PRINT("Altitude: ");
	// PRINT(ext_altitude);
	// PRINTLN(" m above mean sea level");

	PRINTLN("Pressure: " + String(ext_pressure) + " mbar\n" +
        "Temperature: " + String(ext_temp) + " deg C\n" +
        "Depth: " + String(depth) + " m\n" +
        "Altitude: " + String(ext_altitude) + " m above mean sea level");
	
	LOGLN(("Pressure: " + String(ext_pressure) + " mbar\n" +
		"Temperature: " + String(ext_temp) + " deg C\n" +
		"Depth: " + String(depth) + " m\n" +
		"Altitude: " + String(ext_altitude) + " m above mean sea level").c_str());

	yawDepth[1] = depth;
}

/**
 * Function to get BMP280 readings
 */
void bmpRead()
{
	int_temp = bmp.readTemperature();
	int_pressure = bmp.readPressure() / 100;
	int_altitude = bmp.readAltitude(1013.25);
	PRINTLN("\nBMP280 Reading:\nTemperature: " + String(int_temp) + " *C\nPressure: " + String(int_pressure) + " mb\nAltitude: " + String(int_altitude) + " m");
	LOGLN(("\nBMP280 Reading:\nTemperature: " + String(int_temp) + " *C\nPressure: " + String(int_pressure) + " mb\nAltitude: " + String(int_altitude) + " m").c_str());

	intPrTp[0] = int_pressure;
	intPrTp[1] = int_temp;
}

/**
 * Function to get LIS3MDL (MAG) readings
 */
void magRead()
{
	MAG.read(); // get X Y and Z data at once
	// Then print out the raw data
	magX = MAG.x;
	magY = MAG.y;
	magZ = MAG.z;

	PRINTLN("\nMAG Reading:\nX: " + String(magX) + "\tY: " + String(magY) + "\tZ: " + String(magZ));
	LOGLN(("\nMAG Reading:\nX: " + String(magX) + "\tY: " + String(magY) + "\tZ: " + String(magZ)).c_str());

	/* Or....get a new sensor event, normalized to uTesla */
	sensors_event_t event;
	MAG.getEvent(&event);
	/* Display the results (magnetic field is measured in uTesla) */
	
	PRINTLN("X: " + String(event.magnetic.x) + "\tY: " + String(event.magnetic.y) + "\tZ: " + String(event.magnetic.z) + " uTesla");
	LOGLN(("X: " + String(event.magnetic.x) + "\tY: " + String(event.magnetic.y) + "\tZ: " + String(event.magnetic.z) + " uTesla").c_str());

}

/**
 * Function to get MPU6500 readings
 */
void mpuRead()
{
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


	PRINTLN("\nMPU Reading:\nAcceleration in g (x,y,z):\t" + String(accelX) + "   " + String(accelY) + "   " + String(accelZ) + "\nResultant g: " + String(resultantG));
	LOGLN(("\nMPU Reading:\nAcceleration in g (x,y,z):\t" + String(accelX) + "   " + String(accelY) + "   " + String(accelZ) + "\nResultant g: " + String(resultantG)).c_str());
	
	PRINTLN("Gyroscope data in degrees/s:\t" + String(gyroX) + "   " + String(gyroY) + "   " + String(gyroZ));
	LOGLN(("Gyroscope data in degrees/s:\t" + String(gyroX) + "   " + String(gyroY) + "   " + String(gyroZ)).c_str());

	PRINTLN("Temperature in °C: " + String(temp));
	LOGLN(("Temperature in °C: " + String(temp)).c_str());

	PRINTLN("********************************************");
	LOGLN("********************************************");

}

/**
 * Test function to call all 5 reads
 */
void allRead()
{
	PRINTLN();
	// if (INA_C) { inaReadVoltage(); inaReadCurr(); delay(READ_DELAY);}
	if (BAR_C)
	{
		bar30Read();
		delay(READ_DELAY);
	}
	// if (BMP_C) { bmpRead(); delay(READ_DELAY);}
	if (MAG_C)
	{
		magRead();
		delay(READ_DELAY);
	}
	if (MPU_C)
	{
		mpuRead();
		getRollPitch();
		getRollPitchK();
		getYaw();
		delay(READ_DELAY);
	}
	if (enable_CAN)
	{
		// CAN.IMU_low(rollPitch, 0x13, 10);
		// CAN.IMU_hi_depth(yawDepth, 0x14, 10);
		// CAN.PHT(intPrTp, 0x16, 10);
		// CAN.VCS(voltCurr, 0x16, 10);
		// Might be able to change to 0
		CAN.send2data(rollPitch, 0x13);
		CAN.send2data(yawDepth, 0x14);
	}
}

/**
 * Indicator LED off
 */
void turnOff()
{
	digitalWrite(IND_1, LOW);
}

/**
 * Indicator LED flash
 * @param num: How many times the LED fast flashes
 * @param pin: Which LED to flash: 1: Blue, 2: Green
 * @param speed: How fast: 1: Fast, 2: Slow
 */
void flash(int num, int pin, int speed)
{
	uint8_t led = 0;
	int time_delay = 0;
	switch (pin)
	{
	case 1:
		led = IND_1;
		break;
	case 2:
		led = IND_2;
		break;
	}
	switch (speed)
	{
	case 1:
		time_delay = FAST;
		break;
	case 2:
		time_delay = SLOW;
		break;
	}
	for (int i = 0; i < num; i += 1)
	{
		digitalWrite(led, HIGH);
		delay(time_delay);
		digitalWrite(led, LOW);
		delay(time_delay);
	}
}

/**
 * Setup the INA219 power monitoring board
 */
void inaSetup()
{
	int count = 0;
	while (!INA.begin() && count < 10)
	{
		PRINTLN("INA219 init failed!");
		count += 1;
		delay(100);
	}
	if (count < 10)
	{
		PRINTLN("INA219 connected");
		INA_C = true;
	}
	else
	{
		PRINTLN("INA219 NOT connected");
	}
}

/**
 * Read voltage value from INA219 power monitoring board
 */
void inaReadVoltage()
{
	voltage = INA.getBusVoltage_V();
	PRINT("Clean Battery Voltage:\t");
	PRINTLN(voltage);
	voltCurr[0] = voltage;
}

/**
 * Read current value from INA219 power monitoring board
 */
void inaReadCurr()
{
	current = INA.getCurrent_mA();
	PRINT("Clean Battery Current:\t");
	PRINTLN(current);
	voltCurr[1] = current;
}

/**
 * Read power value from INA219 power monitoring board
 */
void inaReadPwr()
{
	float bus_pwr = INA.getPower_mW();
	PRINT("Clean Battery Power:\t");
	PRINTLN(bus_pwr);
}

/**
 * Send power monitoring data
 */
void sendVC()
{
	inaReadVoltage();
	inaReadCurr();
	CAN.send2data(voltCurr, 0x16);
}

/**
 * Software Reboot
 */
void reboot()
{
	SCB_AIRCR = 0x05FA0004;
}

/**
 * Check hardware pin for software reset
 */
void check_reset_pin()
{
	if (digitalRead(RESET) == LOW)
	{
		reboot();
	}
}

/**
 * Check CAN for software reset
 */
void check_reset_CAN()
{
	CAN.SF_reset();
}

/**
 * IMU data to roll/pitch
 */
void getRollPitch()
{

	PRINTLN("Raw AccelXYZ:\t" + String(accelX) + "\t" + String(accelY) + "\t" + String(accelZ));
	LOGLN(("Raw AccelXYZ:\t" + String(accelX) + "\t" + String(accelY) + "\t" + String(accelZ)).c_str());

	PRINTLN("Raw GyroXYZ:\t" + String(gyroX) + "\t" + String(gyroY) + "\t" + String(gyroZ));
	LOGLN(("Raw GyroXYZ:\t" + String(gyroX) + "\t" + String(gyroY) + "\t" + String(gyroZ)).c_str());


	roll = atan((double)accelY / hypotenuse((double)accelX, (double)accelZ)) * RAD_TO_DEG;
	pitch = atan2((double)-accelX, (double)accelZ) * RAD_TO_DEG;
	
	PRINTLN("  Roll: " + String(roll) + "\t  Pitch: " + String(pitch));
	LOGLN(("  Roll: " + String(roll) + "\t  Pitch: " + String(pitch)).c_str());

	// rollPitch[0] = roll;
	// rollPitch[1] = pitch;
}

/**
 * Kalman Filter TKJ setup
 */
void kalmanSetup()
{
	kalmanX.setAngle(roll);
	kalmanY.setAngle(pitch);
	timer = micros();
	kalmanX.setQbias(0.002f);
	kalmanY.setQbias(0.002f);
}

/**
 * Kalman Filter TKJ
 */
void getRollPitchK()
{
	double dt = (double)(micros() - timer) / 1000000;
	timer = micros();
	kalRoll = kalmanX.getAngle(roll, gyroX, dt);
	kalPitch = kalmanY.getAngle(pitch, gyroY, dt);
	PRINTLN("K Roll: " + String(kalRoll) + "\tK Pitch: " + String(kalPitch));
	LOGLN(("K Roll: " + String(kalRoll) + "\tK Pitch: " + String(kalPitch)).c_str());

	rollPitch[0] = kalRoll;
	rollPitch[1] = kalPitch;
}

/**
 * Get yaw values
 */
void getYaw()
{
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

/**
 * Output the magnatometer values to the serial monitor for calibration purpose.
 */
void magCal_withGUI()
{
	MAG.read(); // get X Y and Z data at once
	// Then print out the raw data
	magX = MAG.x;
	magY = MAG.y;
	magZ = MAG.z;

	sensors_event_t event;
	MAG.getEvent(&event);

	PRINTLN("Raw:0,0,0,0,0,0," + String(int(event.magnetic.x * 10)) + "," + String(int(event.magnetic.y * 10)) + "," + String(int(event.magnetic.z * 10)) + "");
	LOGLN(("Raw:0,0,0,0,0,0," + String(int(event.magnetic.x * 10)) + "," + String(int(event.magnetic.y * 10)) + "," + String(int(event.magnetic.z * 10))).c_str());

	PRINTLN("Uni:0,0,0,0,0,0," + String(event.magnetic.x) + "," + String(event.magnetic.y) + "," + String(event.magnetic.z) + "");\
	LOGLN(("Uni:0,0,0,0,0,0," + String(event.magnetic.x) + "," + String(event.magnetic.y) + "," + String(event.magnetic.z)).c_str());
}

void setupSD()
{
	if (!SD.begin(chipSelect))
	{
		PRINTLN("SD card initialization failed!");
		return;
	}
}

void logToSD(const char* message)
{
	File logFile;

	logFile = SD.open("log.txt", FILE_WRITE);
	if (logFile)
	{
		logFile.println(message);
		logFile.close();
		PRINTLN("Message logged to SD card");
	}
	else
	{
		PRINTLN("Error opening log file");
	}
}