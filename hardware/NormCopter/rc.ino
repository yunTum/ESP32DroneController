
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

uint16_t seqno;
volatile boolean gotRC;

const int receivePacketSize = 18;
uint8_t buffer[receivePacketSize];
uint8_t sendBuffer[55];
unsigned int rcUdpPort = 4211;  //  port to listen on
bool success = false;

// int16_t型用
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

// 符号付きfloat値用
void storeRCFloatSigned(float in, uint8_t * out)
{
  uint16_t converted = abs((int16_t)(in * 100.0));
  uint8_t sign = (in < 0) ? 1 : 0;  // 符号情報
  out[0] = sign;  // 符号を別バイトで送信
  out[1] = (converted >> 8) & 0xFF;  // 上位8ビット
  out[2] = converted & 0xFF;         // 下位8ビット
}

void init_RC()
{
  // UDP通信開始
  Udp.begin(localUdpPort);
  Serial.printf("UDP port %d\n", localUdpPort);
}

void rcvalCyclic()
{
  // パケットサイズを取得
  int packetSize = Udp.parsePacket();
  if (packetSize == receivePacketSize)
  {
    Udp.read(rxPacket, receivePacketSize);
    // ヘッダー：0x55
    if (rxPacket[0] == 0x55)
    {
      rcValue[0] = (rxPacket[ 2]<<8) + rxPacket[ 3]; 
      rcValue[1] = (rxPacket[ 4]<<8) + rxPacket[ 5]; 
      rcValue[2] = (rxPacket[ 6]<<8) + rxPacket[ 7]; 
      rcValue[3] = (rxPacket[ 8]<<8) + rxPacket[ 9]; 
      rcValue[4] = (rxPacket[10]<<8) + rxPacket[11]; 
      rcValue[5] = (rxPacket[12]<<8) + rxPacket[13]; 
      rcValue[6] = (rxPacket[14]<<8) + rxPacket[15]; 
      rcValue[7] = (rxPacket[16]<<8) + rxPacket[17]; 
      if (debugvalue == 'r')
      {
        Serial.printf("%4d %4d %4d ",  rcValue[0], rcValue[1], rcValue[2]);
        Serial.printf("%4d %4d %4d ",rcValue[3], rcValue[4], rcValue[5]);
        Serial.printf("%4d %4d \n",rcValue[6], rcValue[7]);
      }
     
      nextRCtime = millis() + 250; // 250ms
      if (rcValue[4] > 1750) armed = true; 
      else                   armed = false; 
      if (rcValue[5] > 1750) fmode = true; 
      else                   fmode = false;
      if (rcValue[6] > 1750) led = true; 
      else                   led = false;
      if (rcValue[7] > 1750) calibrateRequest = true; 
      else                   calibrateRequest = false;
    

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
  }
}

void sendDroneData() 
{
  bool success = Udp.beginPacket("192.168.40.245", rcUdpPort);
  if (!success)
  {
    return;
  }
  // バッファサイズを55バイトに設定
  // 1 byte：0x55
  sendBuffer[0] = 0x55;
  // 2-3 byte：シーケンス番号
  storeRC(seqno++, &sendBuffer[1]);

  // 3-10 byte：サーボ値
  storeRC(servo[0], &sendBuffer[3]);
  storeRC(servo[1], &sendBuffer[5]);
  storeRC(servo[2], &sendBuffer[7]);
  storeRC(servo[3], &sendBuffer[9]);

  // 11-20 byte：PID値
  storeRCFloatSigned(roll_PID, &sendBuffer[11]);
  storeRCFloatSigned(pitch_PID, &sendBuffer[14]);
  storeRCFloatSigned(yaw_PID, &sendBuffer[17]);

  // 20-28 byte：IMU角度
  storeRCFloatSigned(roll_IMU, &sendBuffer[20]);
  storeRCFloatSigned(pitch_IMU, &sendBuffer[23]);
  storeRCFloatSigned(yaw_IMU, &sendBuffer[26]);

  // 29-37 byte：ジャイロ値
  storeRCFloatSigned(GyroX, &sendBuffer[29]);
  storeRCFloatSigned(GyroY, &sendBuffer[32]);
  storeRCFloatSigned(GyroZ, &sendBuffer[35]);  

  // 38-39 byte：バッテリー電圧
  battery_vol = getBattery();
  storeRCFloat(battery_vol, &sendBuffer[38]);

  // 40-41 byte：高度
  storeRCFloat(0, &sendBuffer[40]);

  // 42-43 byte：温度
  storeRCFloat(0, &sendBuffer[42]);

  // 44-45 byte：キャリブレーションリクエスト
  storeRC(calibrateRequest, &sendBuffer[44]);

  // 46-54 byte：加速度値 
  storeRCFloatSigned(AccX, &sendBuffer[46]);
  storeRCFloatSigned(AccY, &sendBuffer[49]);
  storeRCFloatSigned(AccZ, &sendBuffer[52]);

  Udp.write(sendBuffer, 55);  // 55バイト
  Udp.endPacket();

  // bufferをクリア
  memset(sendBuffer, 0, sizeof(sendBuffer));
  success = false;
}
//-----------------------------------------------
