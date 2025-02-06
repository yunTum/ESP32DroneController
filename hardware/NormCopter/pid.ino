
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

float roll_PID = 0;
float pitch_PID = 0;
float yaw_PID = 0;

float i_limit = 5.0; 
extern int16_t rcValue[];

float pid_limit = 50;


// Stableモード用のPIDゲイン
float Kp_stable = 1.3;     // 水平維持用の比例ゲイン
float Ki_stable = 0.04;    // 水平維持用の積分ゲイン
float Kd_stable = 0.02;    // 水平維持用の微分ゲイン

// Stableモード用の積分項
float integral_stable_roll = 0;
float integral_stable_pitch = 0;
float prev_stable_roll_error = 0;
float prev_stable_pitch_error = 0;

#define MAX_ANGLE 15.0     // 最大傾斜角（度）
#define STABLE_I_LIMIT 10.0 // 積分項の制限
#define STABLE_ANGLE 10.0
#define YAW_ANGLE 3.0
void controlStable() 

{
  // ロール角の制御（水平に戻す）
  float roll_error = 0 - roll_IMU;  // 目標角度は0度（水平）
  integral_stable_roll += roll_error * dt;
  integral_stable_roll = constrain(integral_stable_roll, -STABLE_I_LIMIT, STABLE_I_LIMIT);
  float roll_D = (roll_error - prev_stable_roll_error) / dt;
  
  // スティック入力との組み合わせ
  float stick_roll = roll_rc * MAX_ANGLE;
  roll_des = stick_roll + (Kp_stable * roll_error + 
                          Ki_stable * integral_stable_roll + 
                          Kd_stable * roll_D);
  
  // ピッチ角の制御（水平に戻す）
  float pitch_error = 0 - pitch_IMU;  // 目標角度は0度（水平）
  integral_stable_pitch += pitch_error * dt;
  integral_stable_pitch = constrain(integral_stable_pitch, -STABLE_I_LIMIT, STABLE_I_LIMIT);
  float pitch_D = (pitch_error - prev_stable_pitch_error) / dt;
  
  // スティック入力との組み合わせ
  float stick_pitch = pitch_rc * MAX_ANGLE;
  pitch_des =  stick_pitch + (Kp_stable * pitch_error + 
                            Ki_stable * integral_stable_pitch + 
                            Kd_stable * pitch_D);
                            
  // ヨー制御（通常のレートモード）
  yaw_des = yaw_rc * 45.0;  // ヨーレートの目標値（度/秒）
  
  // 前回値の保存
  prev_stable_roll_error = roll_error;
  prev_stable_pitch_error = pitch_error;
  
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
  derivative_roll = (GyroX_prev - GyroX)/dt; 
  roll_PID  = Kp_rate*error_roll;
  roll_PID += Ki_rate*integral_roll; 
  roll_PID += Kd_rate*derivative_roll;
  integral_roll_prev = integral_roll;
  error_roll_prev = error_roll;
  GyroX_prev = GyroX;
  
  //Pitch
  error_pitch = pitch_rate_des - GyroY;
  integral_pitch = integral_pitch_prev + error_pitch*dt;
  if (rcValue[THR] < MINTHROTTLE) integral_pitch = 0;
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup

  // derivative_pitch = (error_pitch - error_pitch_prev)/dt; 
  derivative_pitch = (GyroY_prev - GyroY)/dt; 
  pitch_PID  = Kp_rate*error_pitch;
  pitch_PID += Ki_rate*integral_pitch; 
  pitch_PID += Kd_rate*derivative_pitch;
  integral_pitch_prev = integral_pitch;
  error_pitch_prev = error_pitch;
  GyroY_prev = GyroY;

  //Yaw, stablize on rate from GyroZ
  error_yaw = yaw_rate_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw*dt;
  if (rcValue[THR] < MINTHROTTLE) integral_yaw = 0;
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  // yaw_PID = (Kp_yaw*error_yaw + Ki_yaw*integral_yaw );
  // derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
  derivative_yaw = (GyroZ_prev - GyroZ)/dt; 
  yaw_PID  = Kp_yaw*error_yaw;
  yaw_PID += Ki_yaw*integral_yaw; 
  yaw_PID += Kd_yaw*derivative_yaw;
  integral_yaw_prev = integral_yaw;
  error_yaw_prev = error_yaw;
  GyroZ_prev = GyroZ;

  // limit PID output
  // roll_PID = constrain(roll_PID, -pid_limit, pid_limit);
  // pitch_PID = constrain(pitch_PID,  -pid_limit, pid_limit);
  // yaw_PID = constrain(yaw_PID, -pid_limit, pid_limit);
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
  EEPROM.commit();      
}
