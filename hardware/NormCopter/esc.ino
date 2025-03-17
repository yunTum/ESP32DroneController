uint16_t servo[6];
int maxm = 1800; // 1800 output
#define minm 200 // 200 output

#define THRO_5V_MIN 10
#define THRO_5V_MAX 200
#define THRO_3V3_MIN 50
#define THRO_3V3_MAX 1800

#define YAW_MAX_DELTA 5.0  // 1フレームあたりの最大ヨー変化量

// レート調整
const float powerRate = 1.5;

// モーターごとの補正係数を追加
float motor_correction[] = {
  0.94, // RightBack
  1.02, // RightTop
  0.78, // LeftBack
  0.85  // LeftTop
};  // 必要に応じて調整

// モーターキャリブレーション用の変数
#define CALIB_DURATION 3000  // 各モーターのテスト時間（ミリ秒）
#define CALIB_POWER 300     // キャリブレーション時の出力値

// キャリブレーションモード用の関数
void calibrateMotors() {
  // 安全のため、最初はすべてのモーターを停止
  for(int i = 0; i < 4; i++) {
    servo[i] = 0;
  }
  
  // 各モーターを順番にテスト
  for(int motor = 0; motor < 4; motor++) {
    // 3秒待機（準備時間）
    delay(3000);
    
    // 現在のモーターのみを回転
    servo[motor] = CALIB_POWER;
    // 他のモーターを停止
    for(int i = 0; i < 4; i++) {
      if(i != motor) servo[i] = 0;
    }
    writeMot();
    
    // テスト時間だけ回転
    delay(CALIB_DURATION);
    
    // モーターを停止
    servo[motor] = 0;
    
    // モーター名を表示
    Serial.print("Motor ");
    switch(motor) {
      case 0: Serial.println("RightBack tested"); break;
      case 1: Serial.println("RightTop tested"); break;
      case 2: Serial.println("LeftBack tested"); break;
      case 3: Serial.println("LeftTop tested"); break;
    }
  }
}

// 実際のモーター出力に補正を適用する関数
void applyMotorCorrections() {
  servo[0] = CALIB_POWER * motor_correction[0];  // RightBack
  servo[1] = CALIB_POWER * motor_correction[1];  // RightTop
  servo[2] = CALIB_POWER * motor_correction[2];  // LeftBack
  servo[3] = CALIB_POWER * motor_correction[3];  // LeftTop
  writeMot();
  // テスト時間だけ回転
  delay(CALIB_DURATION);
  servo[0] = 0; servo[1] = 0; servo[2] = 0; servo[3] = 0;
  writeMot();
}

void mix()
{
  if (armed & (rcValue[THR] > MINTHROTTLE))
  {
    int thro = rcValue[THR] - 15;
    float roll_power = constrain(axisPID[ROLL] * 0.7, -20, 20);
    float pitch_power = constrain(axisPID[PITCH] * 0.7, -20, 20); 

    // ヨーの出力をスムージング
    static float prev_yaw_power = 0;
    float yaw_power = constrain(axisPID[YAW] * 0.7, -20, 20);
    // float yaw_delta = yaw_power - prev_yaw_power;
    // yaw_delta = constrain(yaw_delta, -YAW_MAX_DELTA, YAW_MAX_DELTA);
    // yaw_power = prev_yaw_power + yaw_delta;
    // prev_yaw_power = yaw_power;

    float servo_power0 = - roll_power - pitch_power - yaw_power;  // RightBack
    float servo_power1 = - roll_power + pitch_power + yaw_power;  // RightTop
    float servo_power2 = + roll_power - pitch_power + yaw_power;  // LeftBack
    float servo_power3 = + roll_power + pitch_power - yaw_power;  // LeftTop

    float alt_power = constrain(axisPID_alt * 0.5, -20, 20);  // 高度制御の出力

    if(fmode){
      // モーター出力を計算（スロットル値に対する割合で調整）
      servo[0] = constrain(thro + servo_power0, THRO_3V3_MIN, maxm);
      servo[1] = constrain(thro + servo_power1, THRO_3V3_MIN, maxm);
      servo[2] = constrain(thro + servo_power2, THRO_3V3_MIN, maxm);
      servo[3] = constrain(thro + servo_power3, THRO_3V3_MIN, maxm);
    }
    else{
      // スタビライズモード
      servo[0] = constrain(thro + servo_power0 * powerRate, THRO_3V3_MIN, maxm) * motor_correction[0];
      servo[1] = constrain(thro + servo_power1 * powerRate, THRO_3V3_MIN, maxm) * motor_correction[1];
      servo[2] = constrain(thro + servo_power2 * powerRate, THRO_3V3_MIN, maxm) * motor_correction[2];
      servo[3] = constrain(thro + servo_power3 * powerRate, THRO_3V3_MIN, maxm) * motor_correction[3];
    }
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
  // ledcWrite(MotChannel0, servo[0]);
  // ledcWrite(MotChannel1, servo[1]);
  // ledcWrite(MotChannel2, servo[2]);
  // ledcWrite(MotChannel3, servo[3]);
  ledcWrite(MotPin0, servo[0]);
  ledcWrite(MotPin1, servo[1]);
  ledcWrite(MotPin2, servo[2]);
  ledcWrite(MotPin3, servo[3]);
}

void initMot() 
{
  // ledcSetup(MotChannel0, 15000, 11); // 500 hz PWM, 11-bit resolution 720motor
  // ledcSetup(MotChannel1, 15000, 11); // 500 hz PWM, 11-bit resolution 720motor
  // ledcSetup(MotChannel2, 15000, 11); // 500 hz PWM, 11-bit resolution 720motor
  // ledcSetup(MotChannel3, 15000, 11); // 500 hz PWM, 11-bit resolution 720motor
  // ledcSetup(MotChannel0, 20000, 11); // 500 hz PWM, 11-bit resolution
  // ledcSetup(MotChannel1, 20000, 11); // 500 hz PWM, 11-bit resolution
  // ledcSetup(MotChannel2, 20000, 11); // 500 hz PWM, 11-bit resolution
  // ledcSetup(MotChannel3, 20000, 11); // 500 hz PWM, 11-bit resolution
  // ledcAttachPin(MotPin0, MotChannel0); 
  // ledcAttachPin(MotPin1, MotChannel1); 
  // ledcAttachPin(MotPin2, MotChannel2); 
  // ledcAttachPin(MotPin3, MotChannel3); 

  // ESP32のCore V3.0.0以降では、ledcSetupとledcAttachPinの代わりにledcAttachを使用する必要がある
  ledcAttach(MotPin0, 20000, 11);
  ledcAttach(MotPin1, 20000, 11);
  ledcAttach(MotPin2, 20000, 11);
  ledcAttach(MotPin3, 20000, 11);
  // ledcAttachChannel(MotPin0, 20000, 11, MotChannel0);
  // ledcAttachChannel(MotPin1, 20000, 11, MotChannel1);
  // ledcAttachChannel(MotPin2, 20000, 11, MotChannel2);
  // ledcAttachChannel(MotPin3, 20000, 11, MotChannel3);
  // ledcWrite(MotChannel0, THRO_3V3_MIN);
  // ledcWrite(MotChannel1, THRO_3V3_MIN);
  // ledcWrite(MotChannel2, THRO_3V3_MIN);
  // ledcWrite(MotChannel3, THRO_3V3_MIN);
  ledcWrite(MotPin0, THRO_3V3_MIN);
  ledcWrite(MotPin1, THRO_3V3_MIN);
  ledcWrite(MotPin2, THRO_3V3_MIN);
  ledcWrite(MotPin3, THRO_3V3_MIN);

  delay(2000);
  ledcWrite(MotPin0, 0);
  ledcWrite(MotPin1, 0);
  ledcWrite(MotPin2, 0);
  ledcWrite(MotPin3, 0);
}
