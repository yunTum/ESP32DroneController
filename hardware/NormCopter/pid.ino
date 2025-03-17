float Kp_rate = 0.20;    //P-gain - rate mode
float Ki_rate = 0.01;    //I-gain - rate mode
float Kd_rate = 0.01;   //D-gain - rate mode 
float Kp_yaw  = 0.19;    //Yaw P-gain
float Ki_yaw  = 0.01;    //Yaw I-gain
float Kd_yaw  = 0.02;    //Yaw D-gain 

float Kp_ang  = 0.60;    //Ang P-gain
float Ki_ang  = 0.01;    //Ang I-gain
float Kd_ang  = 0.01;    //Ang D-gain
float Kp_ayw  = 0.20;    //Ang Yaw P-gain
float Ki_ayw  = 0.01;    //Ang Yaw I-gain
float Kd_ayw  = 0.07;    //Ang Yaw D-gain

// 高度制御用のPIDゲイン
float Kp_alt = 1.2;    // 高度制御用の比例ゲイン
float Ki_alt = 0.1;    // 高度制御用の積分ゲイン
float Kd_alt = 0.3;    // 高度制御用の微分ゲイン

// 高度制御用の変数
float error_alt;       // 高度誤差
float integral_alt = 0;
float derivative_alt;
float prev_error_alt = 0;
float alt_des;         // 目標高度
float alt_PID = 0;

#define ALT_I_LIMIT 20.0  // 積分項の制限値
#define ALT_PID_LIMIT 50.0 // PID出力の制限値

float thro_des, roll_des, pitch_des, yaw_des; // RC input 0 to 1
float roll_rate_des, pitch_rate_des, yaw_rate_des; // Desired rate commands
float error_roll, integral_roll, integral_roll_prev, derivative_roll;
float error_pitch,integral_pitch,integral_pitch_prev,derivative_pitch;
float error_yaw,  integral_yaw,  integral_yaw_prev,  derivative_yaw;
float error_roll_prev, error_pitch_prev, error_yaw_prev;
float GyroX_prev, GyroY_prev, GyroZ_prev;
float integral_ang_roll,  integral_ang_roll_prev;
float integral_ang_pitch, integral_ang_pitch_prev;
float integral_ang_yaw,   integral_ang_yaw_prev;

float roll_PID = 0; // left positive
float pitch_PID = 0; // down positive
float yaw_PID = 0; // clockwise positive

float i_limit = 5.0; 
extern int16_t rcValue[];

float pid_limit = 100;
float pid_limit_yaw = 100;

float prev_derivative_roll = 0;
float prev_derivative_pitch = 0;
float prev_derivative_yaw = 0;

// Stableモード用のPIDゲイン
float Kp_stable = 1.0;     // 水平維持用の比例ゲイン
float Ki_stable = 0.15;    // 水平維持用の積分ゲイン
float Kd_stable = 0.35;    // 水平維持用の微分ゲイン

// Stableモード用の積分項
float integral_stable_roll = 0;
float integral_stable_pitch = 0;
float prev_stable_roll_error = 0;
float prev_stable_pitch_error = 0;

#define MAX_ANGLE 15.0     // 最大傾斜角（度）
#define STABLE_I_LIMIT 5.0 // 積分項の制限
#define STABLE_ANGLE 10.0
#define YAW_ANGLE 3.0
#define FILTER_GAIN 0.4
#define D_FILTER_GAIN 0.4
#define D_FILTER_STABLE 0.8    // 微分項用フィルター係数
#define ANGLE_FILTER 0.85     // 角度用ローパスフィルター係数

float filtered_roll_IMU = 0;
float filtered_pitch_IMU = 0;
float prev_roll_D = 0;
float prev_pitch_D = 0;

#define YAW_RATE_MAX 30.0  // 最大ヨーレート（度/秒）
#define YAW_EXPO 0.7      // ヨー入力のエクスポ値

void controlStable() 
{
  // 角度のフィルタリング
  filtered_roll_IMU = filtered_roll_IMU * ANGLE_FILTER + roll_IMU * (1 - ANGLE_FILTER);
  filtered_pitch_IMU = filtered_pitch_IMU * ANGLE_FILTER + pitch_IMU * (1 - ANGLE_FILTER);

  // ロール角の制御（水平に戻す）
  float roll_error = 0 - filtered_roll_IMU;
  integral_stable_roll += roll_error * dt;
  integral_stable_roll = constrain(integral_stable_roll, -STABLE_I_LIMIT, STABLE_I_LIMIT);
  float roll_D = (roll_error - prev_stable_roll_error) / dt;
  roll_D = roll_D * (1 - D_FILTER_STABLE) + prev_roll_D * D_FILTER_STABLE;
  
  // スティック入力なしでの水平維持
  roll_des = (Kp_stable * roll_error + 
              Ki_stable * integral_stable_roll + 
              Kd_stable * roll_D);
  
  // ピッチ角の制御（水平に戻す）
  float pitch_error = 0 - filtered_pitch_IMU;
  integral_stable_pitch += pitch_error * dt;
  integral_stable_pitch = constrain(integral_stable_pitch, -STABLE_I_LIMIT, STABLE_I_LIMIT);
  float pitch_D = (pitch_error - prev_stable_pitch_error) / dt;
  pitch_D = pitch_D * (1 - D_FILTER_STABLE) + prev_pitch_D * D_FILTER_STABLE;
  
  // スティック入力なしでの水平維持
  pitch_des = (Kp_stable * pitch_error + 
               Ki_stable * integral_stable_pitch + 
               Kd_stable * pitch_D);
                            
  // ヨー制御（スティック入力なしの場合は0）
  yaw_des = 0;  // 現在の向きを維持
  
  // 前回値の保存
  prev_stable_roll_error = roll_error;
  prev_stable_pitch_error = pitch_error;
  prev_roll_D = roll_D;
  prev_pitch_D = pitch_D;
  
  // スロットルが低い場合は積分項をリセット
  if (rcValue[THR] < MINTHROTTLE) {
    integral_stable_roll = 0;
    integral_stable_pitch = 0;
  }
  
  // 角度制限
  roll_des = constrain(roll_des, -STABLE_ANGLE, STABLE_ANGLE);
  pitch_des = constrain(pitch_des, -STABLE_ANGLE, STABLE_ANGLE);
  yaw_des = constrain(yaw_des, -YAW_ANGLE, YAW_ANGLE);
}

void controlANG() 
{
  // deg err
  error_roll  = roll_des  - roll_IMU; 
  integral_ang_roll = integral_ang_roll_prev + error_roll*dt;
  if (rcValue[THR] < MINTHROTTLE) integral_ang_roll = 0;
  integral_ang_roll = constrain(integral_ang_roll, -i_limit, i_limit); 
  float roll_rate_command = Kp_ang * error_roll +
                           Ki_ang * integral_ang_roll -
                           Kd_ang * GyroX;
  roll_rate_des = roll_rate_command;
  integral_ang_roll_prev = integral_ang_roll;
  

  error_pitch = pitch_des - pitch_IMU;
  integral_ang_pitch = integral_ang_pitch_prev + error_pitch*dt;
  if (rcValue[THR] < MINTHROTTLE) integral_ang_pitch = 0;
  integral_ang_pitch = constrain(integral_ang_pitch, -i_limit, i_limit); 
  float pitch_rate_command = Kp_ang * error_pitch +
                           Ki_ang * integral_ang_pitch -
                           Kd_ang * GyroY;
  pitch_rate_des = pitch_rate_command;
  integral_ang_pitch_prev = integral_ang_pitch;



  error_yaw = yaw_des - yaw_IMU;
  integral_ang_yaw = integral_ang_yaw_prev + error_yaw*dt;
  if (rcValue[THR] < MINTHROTTLE) integral_ang_yaw = 0;
  integral_ang_yaw = constrain(integral_ang_yaw, -i_limit, i_limit); 
  float yaw_rate_command = Kp_ayw * error_yaw +
                           Ki_ayw * integral_ang_yaw -
                           Kd_ayw * GyroZ;
  yaw_rate_des = yaw_rate_command;

  integral_ang_yaw_prev = integral_ang_yaw;
  
  if (debugvalue == 'p')
  {
    Serial.print(error_yaw);        Serial.print("  ");
    //Serial.print(integral_ang_yaw); Serial.print("  ");
    Serial.print(GyroZ);   Serial.print("  ");
    Serial.print(yaw_des); Serial.println();     
  }
}

void controlRATE() 
{
  //Roll
  error_roll = roll_rate_des - GyroX;
  integral_roll = integral_roll_prev + error_roll*dt;
  if (rcValue[THR] < MINTHROTTLE) integral_roll = 0;
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  // derivative_roll = (error_roll - error_roll_prev)/dt; 
  derivative_roll = (GyroX - GyroX_prev)/dt;
  derivative_roll = derivative_roll * (1 - D_FILTER_GAIN) + prev_derivative_roll * D_FILTER_GAIN;

  roll_PID  = Kp_rate*error_roll;
  roll_PID += Ki_rate*integral_roll; 
  roll_PID -= Kd_rate*derivative_roll;
  // anti-windup
  if (roll_PID > pid_limit) {
    integral_roll = integral_roll_prev;  
  }
  // update state
  integral_roll_prev = integral_roll;
  prev_derivative_roll = derivative_roll;
  error_roll_prev = error_roll;
  GyroX = GyroX * (1 - FILTER_GAIN) + GyroX_prev * FILTER_GAIN;
  GyroX_prev = GyroX;
  // limit PID output
  // roll_PID = constrain(roll_PID, -pid_limit, pid_limit);

  //Pitch
  error_pitch = pitch_rate_des - GyroY;
  integral_pitch = integral_pitch_prev + error_pitch*dt;
  if (rcValue[THR] < MINTHROTTLE) integral_pitch = 0;
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  // derivative_pitch = (error_pitch - error_pitch_prev)/dt; 
  derivative_pitch = (GyroY - GyroY_prev)/dt; 
  derivative_pitch = derivative_pitch * (1 - D_FILTER_GAIN) + prev_derivative_pitch * D_FILTER_GAIN;

  pitch_PID  = Kp_rate*error_pitch;
  pitch_PID += Ki_rate*integral_pitch; 
  pitch_PID -= Kd_rate*derivative_pitch;
  // anti-windup
  if (pitch_PID > pid_limit) {
    integral_pitch = integral_pitch_prev;  
  }
  // update state
  integral_pitch_prev = integral_pitch;
  prev_derivative_pitch = derivative_pitch;
  error_pitch_prev = error_pitch;
  GyroY = GyroY * (1 - FILTER_GAIN) + GyroY_prev * FILTER_GAIN;
  GyroY_prev = GyroY;
  // limit PID output
  // pitch_PID = constrain(pitch_PID, -pid_limit, pid_limit);


  //Yaw, stablize on rate from GyroZ
  error_yaw = yaw_rate_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw*dt;
  if (rcValue[THR] < MINTHROTTLE) integral_yaw = 0;
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  // yaw_PID = (Kp_yaw*error_yaw + Ki_yaw*integral_yaw );
  // derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
  derivative_yaw = (GyroZ - GyroZ_prev)/dt; 
  derivative_yaw = derivative_yaw * (1 - D_FILTER_GAIN) + prev_derivative_yaw * D_FILTER_GAIN;

  yaw_PID  = Kp_yaw*error_yaw;
  yaw_PID += Ki_yaw*integral_yaw; 
  yaw_PID -= Kd_yaw*derivative_yaw;
  // anti-windup
  if (yaw_PID > pid_limit) {
    integral_yaw = integral_yaw_prev;  
  }

  // update state
  integral_yaw_prev = integral_yaw;
  prev_derivative_yaw = derivative_yaw;
  error_yaw_prev = error_yaw;
  GyroZ = GyroZ * (1 - FILTER_GAIN) + GyroZ_prev * FILTER_GAIN;
  GyroZ_prev = GyroZ;
  // limit PID output
  // yaw_PID = constrain(yaw_PID, -pid_limit, pid_limit);
}

void controlAltitude() 
{
  // 現在の高度と目標高度の誤差を計算
  error_alt = alt_des - current_altitude;  // current_altitudeは現在の高度

  // 積分項の計算
  integral_alt += error_alt * dt;
  integral_alt = constrain(integral_alt, -ALT_I_LIMIT, ALT_I_LIMIT);

  // 微分項の計算（高度の変化率）
  derivative_alt = (error_alt - prev_error_alt) / dt;
  
  // PID出力の計算
  alt_PID = Kp_alt * error_alt +
            Ki_alt * integral_alt +
            Kd_alt * derivative_alt;
            
  // PID出力を制限
  alt_PID = constrain(alt_PID, -ALT_PID_LIMIT, ALT_PID_LIMIT);

  // スロットルが低い場合は積分項をリセット
  if (rcValue[THR] < MINTHROTTLE) {
    integral_alt = 0;
  }

  // 状態の更新
  prev_error_alt = error_alt;
}

void readpid()
{
  Kp_rate = EEPROM.readFloat( 8);
  Ki_rate = EEPROM.readFloat(12);
  Kd_rate = EEPROM.readFloat(16);
  Kp_yaw  = EEPROM.readFloat(20);
  Ki_yaw  = EEPROM.readFloat(24);
  Kd_yaw  = EEPROM.readFloat(28);
  Kp_ang  = EEPROM.readFloat(32);
  Ki_ang  = EEPROM.readFloat(36);
  Kd_ang  = EEPROM.readFloat(40);
  Kp_ayw  = EEPROM.readFloat(44);
  Ki_ayw  = EEPROM.readFloat(48);
  Kd_ayw  = EEPROM.readFloat(52);
  Kp_stable = EEPROM.readFloat(56);
  Ki_stable = EEPROM.readFloat(60);
  Kd_stable = EEPROM.readFloat(64);
}
void storepid()


{
  EEPROM.writeFloat( 8, Kp_rate);
  EEPROM.writeFloat(12, Ki_rate);
  EEPROM.writeFloat(16, Kd_rate);
  EEPROM.writeFloat(20, Kp_yaw);
  EEPROM.writeFloat(24, Ki_yaw);
  EEPROM.writeFloat(28, Kd_yaw);
  EEPROM.writeFloat(32, Kp_ang);
  EEPROM.writeFloat(36, Ki_ang);
  EEPROM.writeFloat(40, Kd_ang);
  EEPROM.writeFloat(44, Kp_ayw);
  EEPROM.writeFloat(48, Ki_ayw);
  EEPROM.writeFloat(52, Kd_ayw);
  EEPROM.writeFloat(56, Kp_stable);
  EEPROM.writeFloat(60, Ki_stable);
  EEPROM.writeFloat(64, Kd_stable);
  EEPROM.commit();      
}
