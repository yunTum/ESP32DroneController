
uint16_t servo[6];
int maxm = 2500; // for testing
#define minm 200 // for testing

#define THRO_3V3_MIN 50
#define THRO_3V3_MAX 1800

#define PWM_FREQ 30000
// モーターごとの補正係数を追加
// float motor_correction[] = {
//   1.0, // RightBack
//   1.0, // RightTop
//   0.8, // LeftBack
//   0.85  // LeftTop
// };  // 必要に応じて調整
float motor_correction[] = {
  1.0, // RightBack
  0.93, // RightTop
  0.798, // LeftBack
  0.797  // LeftTop
};  // 必要に応じて調整
float powerRate = 0.85;

void mix()
{
  if (armed & (rcValue[THR] > MINTHROTTLE))
  {
    int thro = rcValue[THR] - 15;
    // PID出力をスロットルに対する割合として適用
    float roll_power = axisPID[ROLL] * 1.0;  // ±50を±0.5の範囲に
    float pitch_power = axisPID[PITCH] * 1.0;
    float yaw_power = axisPID[YAW] * 1.0;

    float servo_power0 = - roll_power - pitch_power + yaw_power;  // RightBack
    float servo_power1 = - roll_power + pitch_power - yaw_power;  // RightTop
    float servo_power2 = + roll_power - pitch_power - yaw_power;  // LeftBack
    float servo_power3 = + roll_power + pitch_power + yaw_power;  // LeftTop

    if(fmode){
      // モーター出力を計算（スロットル値に対する割合で調整）
      servo[0] = constrain(thro + servo_power0, THRO_3V3_MIN, maxm);
      servo[1] = constrain(thro + servo_power1, THRO_3V3_MIN, maxm);
      servo[2] = constrain(thro + servo_power2, THRO_3V3_MIN, maxm);
      servo[3] = constrain(thro + servo_power3, THRO_3V3_MIN, maxm);
    }
    else{
      servo[0] = constrain(thro * motor_correction[0] + servo_power0 * powerRate, THRO_3V3_MIN, maxm);
      servo[1] = constrain(thro * motor_correction[1] + servo_power1 * powerRate, THRO_3V3_MIN, maxm);
      servo[2] = constrain(thro * motor_correction[2] + servo_power2 * powerRate, THRO_3V3_MIN, maxm);
      servo[3] = constrain(thro * motor_correction[3] + servo_power3 * powerRate, THRO_3V3_MIN, maxm);
    }
  }
  else 
  { 
    servo[0] = 0; servo[1] = 0; servo[2] = 0; servo[3] = 0;
  }
}

//----------------------------------------------

const int MotPin0 = 32; //12;  //HR mot3 RightTop
const int MotPin1 = 33; //13;  //VR mot2 RightBack
const int MotPin2 = 25;  //15;  //HL mot4 LeftTop
const int MotPin3 = 4; //14;  //VL mot1 LeftBack
const int MotChannel0 = 0;
const int MotChannel1 = 1;   
const int MotChannel2 = 2;
const int MotChannel3 = 3;

void writeMot() 
{
  ledcWrite(MotPin0, servo[0]);
  ledcWrite(MotPin1, servo[1]);
  ledcWrite(MotPin2, servo[2]);
  ledcWrite(MotPin3, servo[3]);
}

void initMot() 
{
  // ESP32のCore V3.0.0以降では、ledcSetupとledcAttachPinの代わりにledcAttachを使用する必要がある
  ledcAttach(MotPin0, PWM_FREQ, 11); // PWM, 11-bit resolution
  ledcAttach(MotPin1, PWM_FREQ, 11); // PWM, 11-bit resolution
  ledcAttach(MotPin2, PWM_FREQ, 11); // PWM, 11-bit resolution
  ledcAttach(MotPin3, PWM_FREQ, 11); // PWM, 11-bit resolution
  ledcWrite(MotPin0, THRO_3V3_MIN);
  delay(1000);
  ledcWrite(MotPin0, 0);
  ledcWrite(MotPin1, THRO_3V3_MIN);
  delay(1000);
  ledcWrite(MotPin1, 0);
  ledcWrite(MotPin2, THRO_3V3_MIN);
  delay(1000);
  ledcWrite(MotPin2, 0);
  ledcWrite(MotPin3, THRO_3V3_MIN);
  delay(1000);
  ledcWrite(MotPin3, 0);
}
