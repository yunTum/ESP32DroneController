#include <WiFi.h>
#include <WebServer.h>
#include "Arduino.h"
#include <EEPROM.h>
#include <esp_wifi.h>
#include <esp_bt.h>
#include <esp_pm.h>

WebServer server(80);

const char* ssid = "XXXXXXXXXXXXXXXXXX";
const char* password = "XXXXXXXXXXXXXXXXXX";

#define MINTHROTTLE 50
#define CALSTEPS 256 // gyro and acc calibration steps
enum ang { ROLL,PITCH,YAW };
#define THR 2 // thro position is rc_value array

#define EEPROM_SIZE 64
#define LED_YELLOW 23
#define LED_RED 18
#define LED_GREEN 5
#define ADC_BAT 35
#define LOW_VOLT 3.0
String act_ip;

extern void initMot();
extern int16_t rcValue[];

#define LoopInterval 4 // minimum 2ms
const float dt = 0.001 * float(LoopInterval); //in sec 
unsigned long synctime;    

// 省電力設定用の定数
#define CPU_FREQ_LOW 80        // 最低CPU周波数(MHz)
#define WIFI_TX_POWER 1        // WiFi送信電力(dBm) 
void setup() 
{
  // WiFi送信電力を下げる（デフォルトは20dBm）
  esp_wifi_set_max_tx_power(WIFI_TX_POWER);
  // CPU周波数を下げる（デフォルトは240MHz）
  setCpuFrequencyMhz(CPU_FREQ_LOW);
  // 自動光パワーセーブモードを有効化
  esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
  // 電源管理の設定
  esp_pm_config_esp32_t pm_config = {
    .max_freq_mhz = CPU_FREQ_LOW,
    .min_freq_mhz = CPU_FREQ_LOW,
    .light_sleep_enable = true
  };
  esp_pm_configure(&pm_config);
  // 未使用のモジュールを無効化
  esp_bt_controller_disable();  // Bluetoothを無効化
  // ADCの解像度を下げる（電力消費を抑える）
  analogSetWidth(10);  // 10ビットに設定


  EEPROM.begin(EEPROM_SIZE);  
  Serial.begin(115200);
  // pinMode(ADC_BAT, INPUT);
  // pinMode(LED_YELLOW, OUTPUT);
  // pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_GREEN, LOW);
  delay(5000);
  // Init Modules
  initMot();
  MPU6050_init();
  readacc();
  // Wi-Fi connection
  WiFi.mode(WIFI_STA);
  WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);
  WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  act_ip = WiFi.localIP().toString().c_str();
  Serial.println(act_ip);

  delay(200);
  init_RC(); 
  server.on("/", handleCmd);
  server.begin();
  Serial.println("HTTP server started");
    
  Serial.println("T - Timing");
  Serial.println("R - RC");
  Serial.println("r - Raw RC");
  Serial.println("G - Gyro raw");
  Serial.println("g - Gyro filt");
  Serial.println("A - Accel raw");
  Serial.println("M - Madgwick");
  Serial.println("P - PID");
  Serial.println("S - Servo");
  Serial.println("B - Battery");

  synctime = millis() + LoopInterval;
  // digitalWrite(LED_YELLOW, LOW);
}

bool armed, fmode;

float GyroX,GyroY,GyroZ;
float AccX, AccY, AccZ;

extern int16_t gyroADC[3];
extern int16_t accADC[3];
extern float roll_IMU, pitch_IMU, yaw_IMU;
extern float roll_PID, pitch_PID, yaw_PID;
extern float roll_des, pitch_des, yaw_des;
extern float roll_rc, pitch_rc, yaw_rc; 
extern int16_t axisPID[3];
extern uint16_t servo[];

unsigned long oldT;
float B_gyro = 0.1;
char debugvalue;
float battery_vol = 0.0;
int counter = 0;
int sendInterval = 100;

void loop() 
{ 
  float gx,gy,gz;
  battery_vol = getBattery();
  if (debugvalue == 'B') showBatteryVol(battery_vol);
  
  // parser part
  if (Serial.available()) 
  {
    char ch = Serial.read();
    if (ch != 10) debugvalue = ch;
  }
  if (debugvalue == 'T') Serial.println(micros() - oldT);
  
  server.handleClient();
  while (millis() < synctime) delay(1);
  synctime += LoopInterval;
  
  if (debugvalue == 'T') oldT = micros();
  
  //des -> thr 0..+300; ang -90..+90
  rcvalCyclic();

  if (debugvalue == 'R') 
    Serial.printf("%4.0f %4.0f %4.0f\n", roll_rc, pitch_rc, yaw_rc);  
  
  roll_des  = roll_rc;
  pitch_des = pitch_rc;
  yaw_des   = yaw_rc;
  
  GyroAcc_getADC(); // 550us instead of 680us
  if (debugvalue == 'G') 
    Serial.printf("%4d %4d %4d \n", gyroADC[0], gyroADC[1], gyroADC[2]);
  if (debugvalue == 'A') 
    Serial.printf("%5d %5d %5d \n", accADC[0], accADC[1], accADC[2]);
    
  // mpu6050 -> 2000 °/s 16.4 LSB/°/s -> 0.061  
  gx = 0.061 * float(gyroADC[0]); 
  gy = 0.061 * float(gyroADC[1]); 
  gz = 0.061 * float(gyroADC[2]); 
  GyroX = (1.0 - B_gyro)*GyroX + B_gyro*gx;
  GyroY = (1.0 - B_gyro)*GyroY + B_gyro*gy;
  GyroZ = (1.0 - B_gyro)*GyroZ + B_gyro*gz;
  if (debugvalue == 'g') 
    Serial.printf("%4.0f %4.0f %4.0f \n", GyroX, GyroY, GyroZ);
  
  // 1g=4096 -> 0.000244
  AccX = 0.01 * float(accADC[0]);  
  AccY = 0.01 * float(accADC[1]);  
  AccZ = 0.01 * float(accADC[2]);
  
  Madgwick6DOF(GyroX,GyroY,GyroZ,AccX,AccY,AccZ,dt);
  if (debugvalue == 'M') 
    Serial.printf("%4.0f %4.0f %4.0f \n", roll_IMU, pitch_IMU, yaw_IMU);

  fmode = true; // test
  if (fmode) controlANG();
  controlRATE();
  
  if (debugvalue == 'P') Serial.printf("%4.0f %4.0f %4.0f \n", roll_PID, pitch_PID, yaw_PID);

  axisPID[ROLL]  = 10.0*roll_PID;
  axisPID[PITCH] = 10.0*pitch_PID;
  axisPID[YAW]   = 10.0*yaw_PID;
  if (counter == sendInterval) {
    sendDroneData();
    counter = 0;
  }
  counter++;
  mix();
  if (debugvalue == 'S') Serial.printf("%4d %4d %4d %4d\n", servo[0], servo[1], servo[2], servo[3]);
  // calib only !
  /*
  if (debugvalue == 'C') 
  {
    servo[0] = 2000; servo[1] = 2000; servo[2] = 2000; servo[3] = 2000;
  }
  if (debugvalue == 'c') 
  {
    servo[0] = 1000; servo[1] = 1000; servo[2] = 1000; servo[3] = 1000;
  }
  /**/
  // if (LOW_VOLT > battery_vol) {
  //   // digitalWrite(LED_GREEN, LOW);
  //   digitalWrite(LED_RED, HIGH);
  //   servo[0] = 0; servo[1] = 0; servo[2] = 0; servo[3] = 0; 
  // } else {
  //   // digitalWrite(LED_GREEN, HIGH);
  //   digitalWrite(LED_RED, LOW);
  // }
  writeMot();
}
