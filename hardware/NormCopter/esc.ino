
uint16_t servo[6];
int maxm = 1800; // for testing
#define minm 200 // for testing

void mix()
{
  /*
  Serial.print(rcValue[THR]);   Serial.print("  ");
  Serial.print(axisPID[ROLL]);  Serial.print("  ");
  Serial.print(axisPID[PITCH]); Serial.print("  ");
  Serial.print(axisPID[YAW]);   Serial.println();
  /**/ 
  
  if (armed & (rcValue[THR] > MINTHROTTLE))
  {
    int thro = rcValue[THR] - 15;
    servo[0] = constrain(thro - axisPID[ROLL] + axisPID[PITCH] - axisPID[YAW],minm,maxm);
    servo[1] = constrain(thro - axisPID[ROLL] - axisPID[PITCH] + axisPID[YAW],minm,maxm);
    servo[2] = constrain(thro + axisPID[ROLL] + axisPID[PITCH] + axisPID[YAW],minm,maxm);
    servo[3] = constrain(thro + axisPID[ROLL] - axisPID[PITCH] - axisPID[YAW],minm,maxm);
  }
  else 
  { 
    //if (!armed) Serial.println("NotArmed");
    servo[0] = 0; servo[1] = 0; servo[2] = 0; servo[3] = 0;
  }
}

//----------------------------------------------

const int MotPin0 = 33; //12;  //HR mot3 RightTop
const int MotPin1 = 32; //13;  //VR mot2 RightBack
const int MotPin2 = 4;  //15;  //HL mot4 LeftTop
const int MotPin3 = 25; //14;  //VL mot1 LeftBacl
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
  ledcSetup(MotChannel0, 15000, 11); // 500 hz PWM, 11-bit resolution
  ledcSetup(MotChannel1, 15000, 11); // 500 hz PWM, 11-bit resolution
  ledcSetup(MotChannel2, 15000, 11); // 500 hz PWM, 11-bit resolution
  ledcSetup(MotChannel3, 15000, 11); // 500 hz PWM, 11-bit resolution
  ledcAttachPin(MotPin0, MotChannel0); 
  ledcAttachPin(MotPin1, MotChannel1); 
  ledcAttachPin(MotPin2, MotChannel2); 
  ledcAttachPin(MotPin3, MotChannel3); 
  ledcWrite(MotChannel0, 50);
  ledcWrite(MotChannel1, 50);
  ledcWrite(MotChannel2, 50);
  ledcWrite(MotChannel3, 50);
}