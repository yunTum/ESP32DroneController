/*
UPDATE MARK 20240108
*/
#include <WiFi.h>
#include <WebServer.h>
#include "Arduino.h"
#include <EEPROM.h>
#include <esp_wifi.h>
#include <esp_bt.h>
#include <esp_pm.h>
#include <Wire.h>

WebServer server(80);

const char* ssid = "****************";
const char* password = "****************";

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
#define CS_PIN 27
String act_ip;

extern void initMot();
extern int16_t rcValue[];

#define LoopInterval 4 // minimum 2ms
const float dt = 0.001 * float(LoopInterval); //in sec 
unsigned long synctime;    

void setup() 
{
  Wire.begin(21, 22);  // SDA=21, SCL=22 (ESP32のデフォルトピン)
  // Wire.setClock(400000); // 400kHz I2C
  Wire.setClock(100000);  // 100kHzに設定

  EEPROM.begin(EEPROM_SIZE);  
  Serial.begin(115200);
  pinMode(ADC_BAT, INPUT);
  pinMode(LED_YELLOW, OUTPUT);
  // pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  digitalWrite(LED_YELLOW, LOW);
  // delay(5000);
  
  // Init Modules
  initMot();
  LSM6DSLTR_init();
  readacc();

  LPSHHTR_init();
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
  Serial.println("H - LPS22HH");
  Serial.println("L - Logging");
  Serial.println("W - PID Settings");

  synctime = millis() + LoopInterval;
  digitalWrite(LED_GREEN, LOW);
  // digitalWrite(LED_YELLOW, LOW);
}

bool armed, fmode, led, calibrateRequest;

float GyroX,GyroY,GyroZ;
float AccX, AccY, AccZ;

extern int16_t gyroADC[3];
extern int16_t accADC[3];
extern float roll_IMU, pitch_IMU, yaw_IMU;
extern float roll_PID, pitch_PID, yaw_PID;
extern float roll_des, pitch_des, yaw_des;
extern float roll_rc, pitch_rc, yaw_rc;
// extern float throttle;
extern int16_t axisPID[3];
extern uint16_t servo[];

extern float pressure, temperature, altitude;

unsigned long oldT;
float B_gyro = 0.1;
char debugvalue;
float battery_vol = 0.0;
int counter = 0;
int sendInterval = 20;

bool isHeader = true;
const int LOG_INTERVAL = 20; // 何回のループに1回ログを取るか
int logCounter = 0;
bool isDetailLog = false;

extern float Kp_rate, Ki_rate, Kd_rate;
extern float Kp_yaw, Ki_yaw, Kd_yaw;
extern float Kp_ang, Ki_ang, Kd_ang;
extern float Kp_ayw, Ki_ayw, Kd_ayw;

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
    Serial.printf("%4.0f %4.0f %4.0f  %d  %d  %d  %d\n", roll_rc, pitch_rc, yaw_rc, armed, fmode, led, calibrateRequest);  
  
  roll_des  = roll_rc;
  pitch_des = pitch_rc;
  yaw_des   = yaw_rc;
  
  if (calibrateRequest) {
    calibrateRequest = false;
    Serial.println("Calibrate Request");
    calibrate();
  }
  
  GyroAcc_getADC(); // 550us instead of 680us

  // gx = float(gyroADC[0]); 
  // gy = float(gyroADC[1]); 
  // gz = float(gyroADC[2]); 

  // GyroX = gx;
  // GyroY = gy;
  // GyroZ = gz;

  // // 1g=4096 -> 0.000244
  // AccX = float(accADC[0]);  
  // AccY = float(accADC[1]);  
  // AccZ = float(accADC[2]);

  if (debugvalue == 'G') 
    Serial.printf("%4d %4d %4d \n", gyroADC[0], gyroADC[1], gyroADC[2]);
  if (debugvalue == 'A') 
    Serial.printf("%5d %5d %5d \n", accADC[0], accADC[1], accADC[2]);
  getLPS22HH();

  if (debugvalue == 'H') 
    Serial.printf("Pressure: %6.2f Temp: %5.2f Alt: %6.2f \n", pressure, temperature, altitude);

  if (debugvalue == 'g') 
    Serial.printf("%4.0f %4.0f %4.0f \n", GyroX, GyroY, GyroZ);
  if (debugvalue == 'a') 
    Serial.printf("%4.0f %4.0f %4.0f \n", AccX, AccY, AccZ);

  Madgwick6DOF(GyroX,GyroY,GyroZ,AccX,AccY,AccZ,dt);
  if (debugvalue == 'M') 
    Serial.printf("%4.0f %4.0f %4.0f \n", roll_IMU, pitch_IMU, yaw_IMU);

  // fmode = true; // test
  if (fmode) {
    controlANG();
  } else {
    controlStable();
  }
  controlRATE();
  
  if (led) digitalWrite(LED_YELLOW, HIGH);
  else      digitalWrite(LED_YELLOW, LOW);

  if (debugvalue == 'P') Serial.printf("%4.0f %4.0f %4.0f \n", roll_PID, pitch_PID, yaw_PID);

  axisPID[ROLL]  = 10.0*roll_PID;
  axisPID[PITCH] = 10.0*pitch_PID;
  axisPID[YAW]   = 10.0*yaw_PID;
  if (counter == sendInterval) {
    sendDroneData();
    counter = 0;
  }

  // ロギング開始/停止のコマンドを追加
  if (debugvalue == 'L') {
    logCounter = 0;
    if(isHeader) {
      // ヘッダー行を送信

      Serial.println("DATA,gyroADC_X,gyroADC_Y,gyroADC_Z,accADC_X,accADC_Y,accADC_Z,GyroX,GyroY,GyroZ,AccX,AccY,AccZ,roll_IMU,pitch_IMU,yaw_IMU,roll_PID,pitch_PID,yaw_PID");
    }
    isHeader = false;
    // データロギング
    if(logCounter++ % LOG_INTERVAL == 0) {
      if (isDetailLog) {
        Serial.printf("DATA,%d,%d,%d,%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
          gyroADC[0], gyroADC[1], gyroADC[2],
          accADC[0], accADC[1], accADC[2],
          GyroX, GyroY, GyroZ,
          AccX, AccY, AccZ,
          roll_IMU, pitch_IMU, yaw_IMU,
          roll_PID, pitch_PID, yaw_PID);
      }
      else {
        Serial.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
          GyroX, GyroY, GyroZ,
          roll_PID, pitch_PID, yaw_PID);
      }
    }
  }

  if (debugvalue == 'W') {
      Serial.println("--------------------------------");
      Serial.println("Rate Settings");
      Serial.println("Kp_rate: " + String(Kp_rate));
      Serial.println("Ki_rate: " + String(Ki_rate));
      Serial.println("Kd_rate: " + String(Kd_rate));
      Serial.println("Yaw Settings");
      Serial.println("Kp_yaw: " + String(Kp_yaw));
      Serial.println("Ki_yaw: " + String(Ki_yaw));
      Serial.println("Kd_yaw: " + String(Kd_yaw));
      Serial.println("Ang Settings");
      Serial.println("Kp_ang: " + String(Kp_ang));
      Serial.println("Ki_ang: " + String(Ki_ang));
      Serial.println("Kd_ang: " + String(Kd_ang));
      Serial.println("Ang-Yaw Settings");
      Serial.println("Kp_ayw: " + String(Kp_ayw));
      Serial.println("Ki_ayw: " + String(Ki_ayw));
      Serial.println("Kd_ayw: " + String(Kd_ayw));
      Serial.println("--------------------------------");
      
      debugvalue = ' ';
  }

  counter++;
  mix();
  if (debugvalue == 'S') Serial.printf("%4d %4d %4d %4d\n", servo[0], servo[1], servo[2], servo[3]);
  // calib only !
  /*
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
