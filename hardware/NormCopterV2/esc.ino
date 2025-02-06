
uint16_t servo[6];
int maxm = 1800; // for testing
#define minm 200 // for testing

#define THRO_3V3_MIN 50
#define THRO_3V3_MAX 1800

#define PWM_FREQ 30000
// モーターごとの補正係数を追加
const float motor_correction[] = {
  1.0, // RightBack
  1.0, // RightTop
  1.0, // LeftBack
  1.0  // LeftTop
};  // 必要に応じて調整
const float powerRate = 1.5;

void mix()
{
  if (armed & (rcValue[THR] > MINTHROTTLE))
  {
    int thro = rcValue[THR] - 15;
    // PID出力をスロットルに対する割合として適用
    float roll_power = axisPID[ROLL] * 1.0;  // ±50を±0.5の範囲に
    float pitch_power = axisPID[PITCH] * 1.0;
    float yaw_power = axisPID[YAW] * 1.0;

    float servo_power0 = - roll_power - pitch_power - yaw_power;
    float servo_power1 = - roll_power + pitch_power + yaw_power;
    float servo_power2 = roll_power - pitch_power + yaw_power;
    float servo_power3 = roll_power + pitch_power - yaw_power;

    if(fmode){
      // モーター出力を計算（スロットル値に対する割合で調整）
      servo[0] = constrain(thro + servo_power0, THRO_3V3_MIN, maxm);
      servo[1] = constrain(thro + servo_power1, THRO_3V3_MIN, maxm);
      servo[2] = constrain(thro + servo_power2, THRO_3V3_MIN, maxm);
      servo[3] = constrain(thro + servo_power3, THRO_3V3_MIN, maxm);
    }
    else{
      servo[0] = constrain(thro + servo_power0 * powerRate, THRO_3V3_MIN, maxm) * motor_correction[0];
      servo[1] = constrain(thro + servo_power1 * powerRate, THRO_3V3_MIN, maxm) * motor_correction[1];
      servo[2] = constrain(thro + servo_power2 * powerRate, THRO_3V3_MIN, maxm) * motor_correction[2];
      servo[3] = constrain(thro + servo_power3 * powerRate, THRO_3V3_MIN, maxm) * motor_correction[3];
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
  ledcWrite(MotChannel0, servo[0]);
  ledcWrite(MotChannel1, servo[1]);
  ledcWrite(MotChannel2, servo[2]);
  ledcWrite(MotChannel3, servo[3]);
}

void initMot() 
{
  ledcSetup(MotChannel0, PWM_FREQ, 11); // 500 hz PWM, 11-bit resolution
  ledcSetup(MotChannel1, PWM_FREQ, 11); // 500 hz PWM, 11-bit resolution
  ledcSetup(MotChannel2, PWM_FREQ, 11); // 500 hz PWM, 11-bit resolution
  ledcSetup(MotChannel3, PWM_FREQ, 11); // 500 hz PWM, 11-bit resolution
  ledcAttachPin(MotPin0, MotChannel0); 
  ledcAttachPin(MotPin1, MotChannel1); 
  ledcAttachPin(MotPin2, MotChannel2); 
  ledcAttachPin(MotPin3, MotChannel3); 
  // ledcWrite(MotChannel0, THRO_3V3_MIN);
  // ledcWrite(MotChannel1, THRO_3V3_MIN);
  // ledcWrite(MotChannel2, THRO_3V3_MIN);
  // ledcWrite(MotChannel3, THRO_3V3_MIN);
}
