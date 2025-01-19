#include "ACAN_T4.h"
#include "ACAN_T4_Settings.h"

class Sense_can{
  public : void Setup_can(int bitrate);
  public : void send2data(float data[2], int addr);
  public : void send1data(float data, int addr);
  public : void SF_reset();
};