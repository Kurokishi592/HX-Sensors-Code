#pragma once

#ifndef _FUNCTIONS_H_
#define _FUNCTIONS_H_

// Misc Stuff
void checkSW();

// All sensor setup
void sensorStart();
void sensorConn();

// BAR30 Setup stuff
void bar30Setup();

// BMP Setup stuff
void bmpSetup();

// MAG Setup stuff
void magSetup();

// MPU6500 Setup stuff
void mpuSetup();
void mpuCalib();

// CAN Setup stuff
void CANSetup();

// Sensor read functions
void bar30Read();
void bmpRead();
void magRead();
void mpuRead();

void allRead();

// Indicator LED functions
void turnOff();
void flash(int, int, int);

// Power Monitoring functions
void inaSetup();
void inaReadVoltage();
void inaReadCurr();
void inaReadPwr();
void sendVC();

// Software Reset
void reboot();
void check_reset_pin();
void check_reset_CAN();

// IMU Conversion
void getRollPitch();
void kalmanSetup();
void getRollPitchK();

// Mag Conversion
void getYaw();
void getBetterYaw();

// Mag Calibration  
void magCal_withGUI();

// SD Card Setup
void setupSD();
void logToSD(const char* message);

void printVisualisation();

#endif