#include <Arduino.h>
#include <Wire.h>
#include "define.h"
#include "functions.h"

void setup() {
	Serial.begin(9600);
	Wire.begin();
	Wire2.begin();
	delay(1000);

	Serial.println("Started");
	pinMode(SW_RD, INPUT_PULLUP);
	pinMode(SW_LD, INPUT_PULLUP);
	pinMode(SW_BD, INPUT_PULLUP);
	pinMode(SW_FD, INPUT_PULLUP);
	pinMode(SW_BAR, INPUT_PULLUP);
	pinMode(SW_BMP, INPUT_PULLUP);
	pinMode(SW_MAG, INPUT_PULLUP);
	pinMode(SW_MPU, INPUT_PULLUP);

	pinMode(IND_1, OUTPUT);
	// pinMode(IND_2, OUTPUT);

  pinMode(RESET, INPUT_PULLUP);

	Serial.println("Pin modes set, checking switches");
	checkSW();
	Serial.println("Switches checked, starting sensors");
	sensorStart();
	Serial.println("Sensor Connected");
	sensorConn();
	Serial.println("DONE, setting up CAN");
	CANSetup();
	Serial.println("CAN started");
	delay(100);
	Serial.println("Flashing LED");
	
	delay(100);
	turnOff();
	Serial.println("LED Off");

	mpuRead();
	getRollPitch();
	kalmanSetup();
}

void loop() {	
	check_reset();
	allRead();
	delay(DELAY_BTWN_READ);
}