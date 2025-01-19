#include "ACAN_T4.h"
#include "ACAN_T4_Settings.h"

class Sense_can{
  public : void Setup_can(int bitrate);
  public : void send2data(float data[2], int addr = 0x13, uint32_t timebuff = 0);
  public : void send1data(float data, int addr = 0x17, uint32_t timebuff = 0);
  public : void Heartbeat();
};