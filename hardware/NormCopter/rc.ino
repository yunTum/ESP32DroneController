
#define CHANNELS 8

int16_t rcValue[CHANNELS];
int16_t axisPID[3];
float roll_rc, pitch_rc, yaw_rc;

unsigned long nextRCtime;    

#define CHANNELS 8
#define PPMIN_CHANNELS 6  // dont raise this
#define RC_IN_PIN 27 

volatile uint32_t last = 0;
volatile uint8_t  chan = 0;
volatile boolean recv;

//-----------------------------------------------

#include <WiFiUdp.h>
WiFiUDP Udp;
unsigned int localUdpPort = 4210;  //  port to listen on
byte rxPacket[32];  // buffer for incoming packets

uint8_t seqno;
volatile boolean gotRC;

const int receivePacketSize = 16;
uint8_t buffer[receivePacketSize];
uint8_t sendBuffer[30];
unsigned int rcUdpPort = 4211;  //  port to listen on
bool success = false;
void storeRC(int16_t in, uint8_t * out)
{
  out[0] = in>>8;
  out[1] = in&0xFF;
}
// float値用
void storeRCFloat(float in, uint8_t * out)
{
  // 電圧を1000倍して整数に変換（例：3.72V → 3720）
  int16_t converted = (int16_t)(in * 1000.0);
  out[0] = converted>>8;
  out[1] = converted&0xFF;
}

void init_RC()
{
  Udp.begin(localUdpPort);
  Serial.printf("UDP port %d\n", localUdpPort);
}

void rcvalCyclic()
{
  int packetSize = Udp.parsePacket();
  if (packetSize == receivePacketSize)
  {
    Udp.read(rxPacket, receivePacketSize);
    if (rxPacket[0] == 0x55)
    {
      rcValue[0] = (rxPacket[ 2]<<8) + rxPacket[ 3]; 
      rcValue[1] = (rxPacket[ 4]<<8) + rxPacket[ 5]; 
      rcValue[2] = (rxPacket[ 6]<<8) + rxPacket[ 7]; 
      rcValue[3] = (rxPacket[ 8]<<8) + rxPacket[ 9]; 
      rcValue[4] = (rxPacket[10]<<8) + rxPacket[11]; 
      rcValue[5] = (rxPacket[12]<<8) + rxPacket[13]; 
      rcValue[6] = (rxPacket[14]<<8) + rxPacket[15]; 
      if (debugvalue == 'r')
      {
        Serial.printf("%4d %4d %4d ",  rcValue[0], rcValue[1], rcValue[2]);
        Serial.printf("%4d %4d %4d \n",rcValue[3], rcValue[4], rcValue[5]);
      }
     
      nextRCtime = millis() + 250; // 250ms
      if (rcValue[4] > 1750) armed = true; 
      else                   armed = false; 
      if (rcValue[5] > 1750) fmode = true; 
      else                   fmode = false;
      if (rcValue[6] > 1750) led = true; 
      else                   led = false;
    
      // roll=0 pitch=1 thr=2 yaw=3  
      roll_rc  = 0.3 * float(rcValue[0]-1515);
      pitch_rc = 0.3 * float(rcValue[1]-1515);
      yaw_rc   = 0.3 * float(rcValue[3]-1515);
      // throttle = rcValue[2];
    }
    else if (nextRCtime < millis()) 
    {
      armed = false;
      Serial.println("RC timeout disarm");
    }
  }
  else 
  {
    Udp.flush(); 
  }
}

void sendDroneData() 
{
  bool success = Udp.beginPacket("XXXXXXXXXXXXXXXXXX", rcUdpPort);
  if (!success)
  {
    return;
  }
  // バッファサイズを30バイトに設定（15個の値 × 2バイト）
  sendBuffer[0] = 0x55;
  sendBuffer[1] = seqno++;
  
  // サーボ値
  storeRC(servo[0], &sendBuffer[2]);
  storeRC(servo[1], &sendBuffer[4]);
  storeRC(servo[2], &sendBuffer[6]);
  storeRC(servo[3], &sendBuffer[8]);
  
  // PID値
  storeRCFloat(roll_PID, &sendBuffer[10]);
  storeRCFloat(pitch_PID, &sendBuffer[12]);
  storeRCFloat(yaw_PID, &sendBuffer[14]);
  
  // IMU角度
  storeRCFloat(roll_IMU, &sendBuffer[16]);
  storeRCFloat(pitch_IMU, &sendBuffer[18]);
  storeRCFloat(yaw_IMU, &sendBuffer[20]);
  
  // ジャイロ値
  storeRCFloat(GyroX, &sendBuffer[22]);
  storeRCFloat(GyroY, &sendBuffer[24]);
  storeRCFloat(GyroZ, &sendBuffer[26]);
  
  // バッテリー電圧
  battery_vol = getBattery();
  storeRCFloat(battery_vol, &sendBuffer[28]);
  Udp.write(sendBuffer, 30);  // 30バイトに変更
  Udp.endPacket();
  // Serial.printf("Send attempt %s at time: %lu Size: %lu\n", 
  //             success ? "SUCCESS" : "FAILED", counter, sizeof(sendBuffer));
  // bufferをクリア
  memset(sendBuffer, 0, sizeof(sendBuffer));
  success = false;
  // delay(10);
}
//-----------------------------------------------

/*
IRAM_ATTR void rxInt() 
{
  uint32_t now,diff; 
    
  now = micros();
  diff = now - last;
  last = now;

  if      (diff > 3000) chan = 0; // Sync gap
  else if (chan < CHANNELS)
  {
    if (950<diff && diff<2050)
    {
      rcValue[chan] = diff;
      chan++;
    }
    else chan = CHANNELS; // skip, corrupted signal.
  }
  if (chan == PPMIN_CHANNELS) recv = true;
}
  
void init_RC()
{
  pinMode(RC_IN_PIN,INPUT);
  attachInterrupt(RC_IN_PIN,rxInt,RISING);
}

void rcvalCyclic()
{
  if (recv)
  {
    if (debugvalue == 'r')
    {
      Serial.printf("%4d %4d %4d ",  rcValue[0], rcValue[1], rcValue[2]);
      Serial.printf("%4d %4d %4d \n",rcValue[3], rcValue[4], rcValue[5]);
    }
    
    recv = false;
    nextRCtime = millis() + 250; // 250ms

    if (rcValue[4] > 1750) armed = true; 
    else                   armed = false; 
    if (rcValue[5] > 1750) fmode = true; 
    else                   fmode = false; 
    
    // roll=0 pitch=1 thr=2 yaw=3  
    roll_rc  = 0.3 * float(rcValue[0]-1515);
    pitch_rc = 0.3 * float(rcValue[1]-1515);
    yaw_rc   = 0.3 * float(rcValue[3]-1515);
  }
  else if (nextRCtime < millis()) 
  {
    armed = false;
    Serial.println("RC timeout disarm");
  }
}
/**/
