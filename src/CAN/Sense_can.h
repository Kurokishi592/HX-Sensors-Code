#pragma once

#ifndef __IMXRT1062__
  #error "Check whether teensy"
#endif

#include "ACAN_T4.h"

class Sense_can{
  public : void Setup_can(int baudrate, int bitrate);
  public : void IMU_low(float data[2], int addr = 0x13, uint32_t timebuff = 2000);
  public : void IMU_hi_depth(float data[2], int addr = 0x14, uint32_t timebuff = 2000);
  // public : void PHT(float data[2], int addr = 0x16, uint32_t timebuff = 2000);
  public : void VCS(float data[2], int addr = 0x16, uint32_t timebuff = 2000);
  public : void Ultra(float data, int addr = 0x17, uint32_t timebuff = 2000);
  public : void Heartbeat(); //done in setup to check ping with jetson
};
