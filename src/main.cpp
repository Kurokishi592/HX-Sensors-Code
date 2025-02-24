#include <Arduino.h>
#include <Wire.h>
#include "define.h"
#include "functions.h"

int count = 0;

void setup() {
	Serial.begin(9600);
	Wire.begin();
	Wire2.begin();
	delay(1000);

	Serial.println("Started");
	// pinMode(SW_RD, INPUT_PULLUP);
	pinMode(SW_PRINTLN, INPUT_PULLUP);
	pinMode(SW_SDLOG, INPUT_PULLUP);
	//pinMode(SW_BD, INPUT_PULLUP); wait for it to be fized
	pinMode(SW_FD, INPUT_PULLUP);
	pinMode(SW_BAR, INPUT_PULLUP);
	pinMode(SW_BMP, INPUT_PULLUP);
	pinMode(SW_MAG, INPUT_PULLUP);
	pinMode(SW_MPU, INPUT_PULLUP);

	pinMode(IND_1, OUTPUT);
	// pinMode(IND_2, OUTPUT);

	pinMode(RESET, INPUT_PULLUP);

	// Serial.println("Pin modes set, checking switches");
	checkSW();
	// Serial.println("Switches checked, starting sensors");
	sensorStart();
	// Serial.println("Sensor Connected");
	sensorConn();
	// Serial.println("DONE, setting up CAN");
	CANSetup();
	// Serial.println("CAN started");
	//setupSD();
	//logToSD("HELLO");
	// Serial.println("SDLOG is enabled");
	delay(100);
	// Serial.println("Flashing LED");
	mpuRead();
	getRollPitch();
	kalmanSetup();
}

void loop() {
	if (count % 50 == 0) {
		check_reset_CAN();
	}
	if (count % 500 == 0) {
		
	}
	if (count == 1000) {
		count = 0;
	}
	allRead();
	delay(DELAY_BTWN_READ);
	count++;

	Serial.println("Looping"); // For me to count number of loops
}