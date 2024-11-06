
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
uint32_t sendInterval = 50;
uint32_t lastRC; 
uint32_t nextSend; 

uint8_t buffer[14];
unsigned int rcUdpPort = 4211;  //  port to listen on

void storeRC(int16_t in, uint8_t * out)
{
  out[0] = in>>8;
  out[1] = in&0xFF;
}

void init_RC()
{
  Udp.begin(localUdpPort);
  Serial.printf("UDP port %d\n", localUdpPort);
}

void rcvalCyclic()
{
  int packetSize = Udp.parsePacket();
  if (packetSize == 14)
  {
    Udp.read(rxPacket, 14);
    if (rxPacket[0] == 0x55)
    {
      rcValue[0] = (rxPacket[ 2]<<8) + rxPacket[ 3]; 
      rcValue[1] = (rxPacket[ 4]<<8) + rxPacket[ 5]; 
      rcValue[2] = (rxPacket[ 6]<<8) + rxPacket[ 7]; 
      rcValue[3] = (rxPacket[ 8]<<8) + rxPacket[ 9]; 
      rcValue[4] = (rxPacket[10]<<8) + rxPacket[11]; 
      rcValue[5] = (rxPacket[12]<<8) + rxPacket[13]; 
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
  else 
  {
    Udp.flush(); 
    // Serial.println("Wrong UDP size");
  }
}

// void sendDroneData() 
// {
//   uint32_t now; 
//   delay(2);
//   now = millis(); // actual time
//   gotRC = true;
//   if (gotRC)
//   {
//     gotRC = false;
//     lastRC = now;
    
//     buffer[ 0] = 0x55;
//     buffer[ 1] = seqno++;
//     storeRC(rcValue[1],&buffer[ 2]);
//     storeRC(rcValue[2],&buffer[ 4]);
//     storeRC(rcValue[0],&buffer[ 6]);
//     storeRC(rcValue[3],&buffer[ 8]);
//     storeRC(rcValue[4],&buffer[10]);
//     storeRC(rcValue[5],&buffer[12]);

//     if (now >= nextSend)
//     {
//       nextSend = now + sendInterval;
//       Udp.beginPacket(WiFi.localIP(), localUdpPort);
//       Serial.printf("%d\n", buffer);
//       Udp.write(buffer, 14);
//       Udp.endPacket();
//     }
//     digitalWrite(LED_YELLOW, HIGH);
//   }
//   else if (now >= lastRC + 250)
//   {
//     lastRC = now;
//     digitalWrite(LED_YELLOW, !digitalRead(LED_YELLOW)); 
//   }
// }
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
