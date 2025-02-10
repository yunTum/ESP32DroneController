const int VOLT = 3.7; // 3.7V固定
// アナログ入力の最大値
const int ANALOG_MAX = 2048.0;

float getBattery() {
  int reading = analogRead(ADC_BAT);
  float voltage = ((float)reading * VOLT ) / ANALOG_MAX;
  return voltage;
}

// シリアル出力
void showBatteryVol(float voltage) {
  Serial.print( "  Volt: " );
  Serial.println( voltage );
}