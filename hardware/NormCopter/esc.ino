
uint16_t servo[6];
int maxm = 1800; // for testing
#define minm 200 // for testing

#define THRO_5V_MIN 10
#define THRO_5V_MAX 200

#define THRO_3V3_MIN 50
#define THRO_3V3_MAX 1800
const float powerRate = 1.5;
// スムージング用の変数を追加
float prev_servo_power[4] = {0, 0, 0, 0};
#define SMOOTH_FACTOR 0.05  // スムージング係数（0.0-1.0）
#define YAW_MAX_DELTA 5.0  // 1フレームあたりの最大ヨー変化量

void mix()
{
  // モーターごとの補正係数を追加
  const float motor_correction[] = {
    1.0, // RightBack
    0.9, // RightTop
    0.8, // LeftBack
    0.9  // LeftTop
    };  // 必要に応じて調整
  if (armed & (rcValue[THR] > MINTHROTTLE))
  {
    int thro = rcValue[THR] - 15;
    // 各モーターの値を計算し、10-100の範囲にマッピング
    // int raw0 = constrain(thro - axisPID[ROLL] - axisPID[PITCH] - axisPID[YAW], minm, maxm) * motor_correction[0];
    // int raw1 = constrain(thro - axisPID[ROLL] + axisPID[PITCH] + axisPID[YAW], minm, maxm) * motor_correction[1];
    // int raw2 = constrain(thro + axisPID[ROLL] - axisPID[PITCH] + axisPID[YAW], minm, maxm) * motor_correction[2];
    // int raw3 = constrain(thro + axisPID[ROLL] + axisPID[PITCH] - axisPID[YAW], minm, maxm) * motor_correction[3];
    
    // servo[0] = map(raw0, minm, maxm, THRO_3V3_MIN, THRO_3V3_MAX);
    // servo[1] = map(raw1, minm, maxm, THRO_3V3_MIN, THRO_3V3_MAX);
    // servo[2] = map(raw2, minm, maxm, THRO_3V3_MIN, THRO_3V3_MAX);
    // servo[3] = map(raw3, minm, maxm, THRO_3V3_MIN, THRO_3V3_MAX);
    
    // servo[0] = constrain(thro - axisPID[ROLL] - axisPID[PITCH] - axisPID[YAW],THRO_3V3_MIN,maxm) * motor_correction[0]; // RightBack
    // servo[1] = constrain(thro - axisPID[ROLL] + axisPID[PITCH] + axisPID[YAW],THRO_3V3_MIN,maxm) * motor_correction[1]; // RightTop
    // servo[2] = constrain(thro + axisPID[ROLL] - axisPID[PITCH] + axisPID[YAW],THRO_3V3_MIN,maxm) * motor_correction[2]; // LeftBack
    // servo[3] = constrain(thro + axisPID[ROLL] + axisPID[PITCH] - axisPID[YAW],THRO_3V3_MIN,maxm) * motor_correction[3]; // LeftTop

    // PID出力をスロットルに対する割合として適用
    // float roll_power = axisPID[ROLL] * 1.0;  // ±50を±0.5の範囲に
    // float pitch_power = axisPID[PITCH] * 1.0;
    float roll_power = constrain(axisPID[ROLL] * 0.7, -20, 20);
    float pitch_power = constrain(axisPID[PITCH] * 0.7, -20, 20); 
    // float yaw_power = axisPID[YAW] * 1.0;
    static float prev_yaw_power = 0;

    float yaw_power = constrain(axisPID[YAW] * 0.3, -15, 15);
    float yaw_delta = yaw_power - prev_yaw_power;
    yaw_delta = constrain(yaw_delta, -YAW_MAX_DELTA, YAW_MAX_DELTA);
    yaw_power = prev_yaw_power + yaw_delta;
    prev_yaw_power = yaw_power;


    float servo_power0 = - roll_power - pitch_power - yaw_power;  // RightBack
    float servo_power1 = - roll_power + pitch_power + yaw_power;  // RightTop
    float servo_power2 = + roll_power - pitch_power + yaw_power;  // LeftBack
    float servo_power3 = + roll_power + pitch_power - yaw_power;  // LeftTop

    // 出力のスムージング処理
    // servo_power0 = prev_servo_power[0] * (1 - SMOOTH_FACTOR) + servo_power0 * SMOOTH_FACTOR;
    // servo_power1 = prev_servo_power[1] * (1 - SMOOTH_FACTOR) + servo_power1 * SMOOTH_FACTOR;
    // servo_power2 = prev_servo_power[2] * (1 - SMOOTH_FACTOR) + servo_power2 * SMOOTH_FACTOR;
    // servo_power3 = prev_servo_power[3] * (1 - SMOOTH_FACTOR) + servo_power3 * SMOOTH_FACTOR;


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
      // for(int i = 0; i < 4; i++) prev_servo_power[i] = 0;
    }



    // servo[0] = thro;


    // servo[1] = thro;
    // servo[2] = thro+200;
    // servo[3] = thro+200;
  }
  else 
  { 
    //if (!armed) Serial.println("NotArmed");
    servo[0] = 0; servo[1] = 0; servo[2] = 0; servo[3] = 0;
  }
}

//----------------------------------------------

const int MotPin0 = 33; //12;  //HR mot3 RightBack
const int MotPin1 = 32; //13;  //VR mot2 RightTop
const int MotPin2 = 4;  //15;  //HL mot4 LeftBack
const int MotPin3 = 25; //14;  //VL mot1 LeftTop
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
  // ledcSetup(MotChannel0, 15000, 11); // 500 hz PWM, 11-bit resolution 720motor
  // ledcSetup(MotChannel1, 15000, 11); // 500 hz PWM, 11-bit resolution 720motor
  // ledcSetup(MotChannel2, 15000, 11); // 500 hz PWM, 11-bit resolution 720motor
  // ledcSetup(MotChannel3, 15000, 11); // 500 hz PWM, 11-bit resolution 720motor
  ledcSetup(MotChannel0, 20000, 11); // 500 hz PWM, 11-bit resolution
  ledcSetup(MotChannel1, 20000, 11); // 500 hz PWM, 11-bit resolution
  ledcSetup(MotChannel2, 20000, 11); // 500 hz PWM, 11-bit resolution
  ledcSetup(MotChannel3, 20000, 11); // 500 hz PWM, 11-bit resolution
  ledcAttachPin(MotPin0, MotChannel0); 
  ledcAttachPin(MotPin1, MotChannel1); 
  ledcAttachPin(MotPin2, MotChannel2); 
  ledcAttachPin(MotPin3, MotChannel3); 
  ledcWrite(MotChannel0, THRO_3V3_MIN);
  ledcWrite(MotChannel1, THRO_3V3_MIN);
  ledcWrite(MotChannel2, THRO_3V3_MIN);
  ledcWrite(MotChannel3, THRO_3V3_MIN);
}
