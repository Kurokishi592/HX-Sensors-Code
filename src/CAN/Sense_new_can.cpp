#include "Sense_new_can.h"
CANMessage msg;
const ACANPrimaryFilter filters[] = { //can just add on if any more filters needed
    ACANPrimaryFilter(kData, kStandard, 0x25)
};

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

void Sense_can :: send2data(float data[2], int addr){ //as sensor is sending float (32 bits), at max 2 32 bit values can be sent per msg
  // create canmessage instance
  msg.id = addr; 
  msg.len = 8;
  msg.dataFloat[0] = data[0]; 
  msg.dataFloat[1] = data[1]; 
  const bool check = ACAN_T4::can1.tryToSend(msg); //send message
  if (!check){
    //SCB_AIRCR = 0x05FA0004;//may want to reset or smth if can't send. just uncomment
  }
}

void Sense_can :: send1data(float data, int addr){ // send 1 data in msg
  msg.id = addr; 
  msg.len = 8;
  msg.dataFloat[0] = data; 
  const bool check = ACAN_T4::can1.tryToSend(msg); //send message
  if (!check){
    //SCB_AIRCR = 0x05FA0004;//may want to reset or smth if can't send just uncomment
  }
}

void Sense_can :: SF_reset(){ //receive only standard frame
  if(ACAN_T4::can1.receive(msg)){
    if (!msg.rtr && !msg.ext && (msg.id == 0x25)) {
      SCB_AIRCR = 0x05FA0004;
    }
  }
}