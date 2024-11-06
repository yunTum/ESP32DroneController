const int VOLT = 3.7; // 3.7V
const int ANALOG_MAX = 2048.0;

float getBattery() {
  int reading = analogRead(ADC_BAT);
  float voltage = ((float)reading * VOLT ) / ANALOG_MAX;
  return voltage;
}

void showBatteryVol(float voltage) {
  Serial.print( "  Volt: " );
  Serial.println( voltage );
}