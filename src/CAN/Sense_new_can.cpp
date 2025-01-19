#include "Sense_new_can.h"

void Sense_can :: Setup_can(int bitrate){
  pinMode (LED_BUILTIN, OUTPUT) ;
  ACAN_T4_Settings settings (bitrate) ; // 250 kbit/s
  const uint32_t errorCode = ACAN_T4::can1.begin (settings) ;

  if (0 == errorCode) {
    digitalWrite(LED_BUILTIN, HIGH);
  }else{
    while (1) {
      delay (5000) ;
      digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ; //blinking means can is not connected
    }
  }
}

void Sense_can :: send2data(float data[2], int addr, uint32_t timebuff ){ //as sensor is sending float (32 bits), at max 2 32 bit values can be sent per msg
  // create canmessage instance
  CANMessage msg;
  msg.id = addr; 
  msg.len = 8;
  msg.dataFloat[0] = data[0]; 
  msg.dataFloat[1] = data[1]; 
  const bool check = ACAN_T4::can1.tryToSend(msg); //send message
}

void Sense_can :: send1data(float data, int addr , uint32_t timebuff){ // send 1 data in msg
  CANMessage msg;
  msg.id = addr; 
  msg.len = 8;
  msg.dataFloat[0] = data; 
  const bool check = ACAN_T4::can1.tryToSend(msg); //send message
}

void Sense_can :: Heartbeat(){ //not too sure why need a unique identifier
  const ACANPrimaryFilter filters[] = {
    ACANPrimaryFilter(kData, kStandard, 0x30)
  };
  CANMessage ping;
  ping.id = 0x10;
  ping.len = 1;
  ping.data[0] = 0x10;
  const bool check = ACAN_T4::can1.tryToSend(ping);
  if(ACAN_T4::can1.receive(ping)){
    digitalWrite(LED_BUILTIN,HIGH);
  }
}