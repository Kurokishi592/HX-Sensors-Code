#include <Arduino.h>
#include <SPI.h>

// #include <ACAN_T4.h>
#include "CAN/Sense_can.h"

#ifndef __IMXRT1062__
  #error "This sketch should be compiled for Teensy 4.x"
#endif

void setup() {
  Sense_can startup;
  startup.Setup_can(9600, 250 * 1000);
}

void loop() {

}