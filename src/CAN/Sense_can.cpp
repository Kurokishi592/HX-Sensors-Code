#include "Sense_can.h"
#include "ACAN_T4.h"
#include "ACAN_T4_Settings.h"

void Sense_can :: Setup_can(int baudrate, int bitrate){
pinMode (LED_BUILTIN, OUTPUT) ;
  // Serial.begin (baudrate) ;
  // while (!Serial) {
  //   delay (50) ;
  //   digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
  // }
  Serial.println ("Initiate CAN settings") ;
  ACAN_T4_Settings settings (bitrate) ; // 250 kbit/s
  const uint32_t errorCode = ACAN_T4::can1.begin (settings) ;

  Serial.print ("Bitrate prescaler: ") ;
  Serial.println (settings.mBitRatePrescaler) ;
  Serial.print ("Propagation Segment: ") ;
  Serial.println (settings.mPropagationSegment) ;
  Serial.print ("Phase segment 1: ") ;
  Serial.println (settings.mPhaseSegment1) ;
  Serial.print ("Phase segment 2: ") ;
  Serial.println (settings.mPhaseSegment2) ;
  Serial.print ("RJW: ") ;
  Serial.println (settings.mRJW) ;
  Serial.print ("Triple Sampling: ") ;
  Serial.println (settings.mTripleSampling ? "yes" : "no") ;
  Serial.print ("Actual bitrate: ") ;
  Serial.print (settings.actualBitRate ()) ;
  Serial.println (" bit/s") ;
  Serial.print ("Exact bitrate ? ") ;
  Serial.println (settings.exactBitRate () ? "yes" : "no") ; 
  Serial.print ("Distance from wished bitrate: ") ;
  Serial.print (settings.ppmFromWishedBitRate ()) ;
  Serial.println (" ppm") ;
  Serial.print ("Sample point: ") ;
  Serial.print (settings.samplePointFromBitStart ()) ;
  Serial.println ("%") ;
  if (0 == errorCode) {
    Serial.println ("can1 ok") ;
  }else{
    Serial.print ("Error can1: 0x") ;
    Serial.println (errorCode, HEX) ;
    while (1) {
      delay (5000) ;
      Serial.println ("Invalid setting") ;   
      digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
    }
  }
}
void Sense_can :: IMU_low(float data[2], int addr, uint32_t timebuff ){
  // create canmessage instance
  CANMessage IMU_low_msg;
  IMU_low_msg.id = addr; 
  IMU_low_msg.len = 8;
  IMU_low_msg.dataFloat[0] = data[0]; //roll 
  IMU_low_msg.dataFloat[1] = data[1]; //pitch
  const bool check = ACAN_T4::can1.tryToSend(IMU_low_msg); //send message
  if (check){
    Serial.print("Roll: ");
    //Serial.printf("%x ", IMU_low_msg.dataFloat[0]);
    Serial.print(IMU_low_msg.dataFloat[0]);
    Serial.print(" Pitch: ");
    //Serial.printf("%x ", IMU_low_msg.dataFloat[1]);
    Serial.print(IMU_low_msg.dataFloat[1]);
    Serial.println("");
    delay(timebuff);
  }
}
void Sense_can :: IMU_hi_depth(float data[2], int addr, uint32_t timebuff){
  //create canmessage instance
  CANMessage IMU_hi_depth_msg;
  IMU_hi_depth_msg.id = addr; 
  IMU_hi_depth_msg.len = 8;
  IMU_hi_depth_msg.dataFloat[0] = data[0]; //yaw
  IMU_hi_depth_msg.dataFloat[1] = data[1]; //depth
  const bool check = ACAN_T4::can1.tryToSend(IMU_hi_depth_msg); //send message
  if (check){
    Serial.print("Yaw: ");
    //Serial.printf("%x ", IMU_hi_depth_msg.dataFloat[0]);
    Serial.print(IMU_hi_depth_msg.dataFloat[0]);
    Serial.print(" Depth: ");
    //Serial.printf("%x ", IMU_hi_depth_msg.dataFloat[1]);
    Serial.print(IMU_hi_depth_msg.dataFloat[1]);
    Serial.println("");
    delay(timebuff);
  }
}

// void Sense_can :: PHT(float data[2], int addr, uint32_t timebuff){
//   CANMessage PHTmsg;
//     PHTmsg.id = addr; 
//     PHTmsg.len = 8;
//     PHTmsg.dataFloat[0] = data[0]; //Pressure
//     PHTmsg.dataFloat[1] = data[1]; //Temperature

//   const bool check = ACAN_T4::can1.tryToSend(PHTmsg); //send message
//   if (check){
//     Serial.print("Pressure: ");
//     Serial.printf("%x ", PHTmsg.dataFloat[0]);
//     Serial.print("Temperature: ");
//     Serial.printf("%x ", PHTmsg.dataFloat[1]);
//     Serial.print("");
//     delay(timebuff);
//   }
// }

void Sense_can :: VCS(float data[2], int addr, uint32_t timebuff){
  CANMessage VCSmsg;
    VCSmsg.id = addr; 
    VCSmsg.len = 8;
    VCSmsg.dataFloat[0] = data[0]; // Voltage
    VCSmsg.dataFloat[1] = data[1]; // Current

  const bool check = ACAN_T4::can1.tryToSend(VCSmsg); //send message
  if (check){
    Serial.print("Voltage: ");
    // Serial.printf("%x ", VCSmsg.dataFloat[0]);
    Serial.print(VCSmsg.dataFloat[0]);
    Serial.print("Current: ");
    Serial.print(VCSmsg.dataFloat[1]);
    Serial.println("");
    delay(timebuff);
  }
}

void Sense_can :: Ultra(float data, int addr , uint32_t timebuff){
  CANMessage Ultramsg;
  Ultramsg.id = addr; 
  Ultramsg.len = 8;
  Ultramsg.dataFloat[0] = data; //Ultrasonic
  const bool check = ACAN_T4::can1.tryToSend(Ultramsg); //send message
  if (check){
    Serial.print("Distance: ");
    Serial.printf("%x ", Ultramsg.dataFloat);
    delay(timebuff);
  }
}

void Sense_can :: Heartbeat(){
  CANMessage ping;
  ping.id = 0x30;
  ping.len = 1;
  ping.data[0] = 0x10;
  const bool check = ACAN_T4::can1.tryToSend(ping);
  if (check){
    Serial.println("Sent successfully");
  }
  if(ACAN_T4::can1.receive(ping)){
    Serial.println("Received successfully");
  }
}
