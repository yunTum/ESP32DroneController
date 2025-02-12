#include <Wire.h>

// BME280のI2Cアドレス
#define BME280_ADDRESS 0x76

// BME280のレジスタアドレス
#define BME280_REGISTER_DIG_T1 0x88
#define BME280_REGISTER_DIG_P1 0x8E
#define BME280_REGISTER_DIG_H1 0xA1
#define BME280_REGISTER_DIG_H2 0xE1
#define BME280_REGISTER_CHIPID 0xD0
#define BME280_REGISTER_CONTROL_HUM 0xF2
#define BME280_REGISTER_CONTROL 0xF4
#define BME280_REGISTER_CONFIG 0xF5
#define BME280_REGISTER_DATA 0xF7
#define BME280_WHO_AM_I 0x60

// 高度計算用の定数
#define SEALEVEL_PRESSURE 1013.25f  // 海面気圧の標準値（hPa）
#define PRESSURE_SCALING -44330.0f   // 高度計算用スケーリング係数

// キャリブレーションデータ
uint16_t dig_T1;
int16_t dig_T2, dig_T3;
uint16_t dig_P1;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
uint8_t dig_H1, dig_H3;
int16_t dig_H2, dig_H4, dig_H5;
int8_t dig_H6;
int32_t t_fine;

void bme280_init() {  
  // チップIDの確認
  if (read8(BME280_REGISTER_CHIPID) != BME280_WHO_AM_I) {
    Serial.println("BME280が見つかりません");
    while (1);
  }

  // キャリブレーションデータの読み取り
  dig_T1 = read16_LE(BME280_REGISTER_DIG_T1);
  dig_T2 = readS16_LE(BME280_REGISTER_DIG_T1 + 2);
  dig_T3 = readS16_LE(BME280_REGISTER_DIG_T1 + 4);

  dig_P1 = read16_LE(BME280_REGISTER_DIG_P1);
  dig_P2 = readS16_LE(BME280_REGISTER_DIG_P1 + 2);
  dig_P3 = readS16_LE(BME280_REGISTER_DIG_P1 + 4);
  dig_P4 = readS16_LE(BME280_REGISTER_DIG_P1 + 6);
  dig_P5 = readS16_LE(BME280_REGISTER_DIG_P1 + 8);
  dig_P6 = readS16_LE(BME280_REGISTER_DIG_P1 + 10);
  dig_P7 = readS16_LE(BME280_REGISTER_DIG_P1 + 12);
  dig_P8 = readS16_LE(BME280_REGISTER_DIG_P1 + 14);
  dig_P9 = readS16_LE(BME280_REGISTER_DIG_P1 + 16);

  dig_H1 = read8(BME280_REGISTER_DIG_H1);
  dig_H2 = readS16_LE(BME280_REGISTER_DIG_H2);
  dig_H3 = read8(BME280_REGISTER_DIG_H2 + 2);
  dig_H4 = ((int8_t)read8(BME280_REGISTER_DIG_H2 + 3) << 4) | (read8(BME280_REGISTER_DIG_H2 + 4) & 0xF);
  dig_H5 = ((int8_t)read8(BME280_REGISTER_DIG_H2 + 5) << 4) | (read8(BME280_REGISTER_DIG_H2 + 4) >> 4);
  dig_H6 = (int8_t)read8(BME280_REGISTER_DIG_H2 + 6);

  // センサーの設定
  write8(BME280_REGISTER_CONTROL_HUM, 0x01);  // 湿度オーバーサンプリング×1
  write8(BME280_REGISTER_CONTROL, 0x27);      // 温度・気圧オーバーサンプリング×1, ノーマルモード
  write8(BME280_REGISTER_CONFIG, 0xA0);       // スタンバイ時間1000ms, フィルタオフ
}

void bme280_read_data(float *temperature, float *pressure, float *humidity, float *altitude) {
  // 生データの読み取り
  uint8_t data[8];
  readRegister(BME280_REGISTER_DATA, data, 8);
  
  int32_t adc_T = ((data[3] << 16) | (data[4] << 8) | data[5]) >> 4;
  int32_t adc_P = ((data[0] << 16) | (data[1] << 8) | data[2]) >> 4;
  int32_t adc_H = (data[6] << 8) | data[7];

  // 温度の計算
  int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
  t_fine = var1 + var2;
  *temperature = (t_fine * 5 + 128) >> 8;
  *temperature /= 100.0f;

  // 気圧の計算
  int64_t p = 1048576 - adc_P;
  p = ((p << 31) - ((int64_t)var2 * 2147483648)) / var1;
  var1 = ((int64_t)dig_P9 * (p >> 13) * (p >> 13)) >> 25;
  var2 = ((int64_t)dig_P8 * p) >> 19;
  p = ((p + var1 + var2) >> 8) + ((int64_t)dig_P7 << 4);
  *pressure = (float)p / 256.0f / 100.0f;

  // 湿度の計算
  int32_t h = t_fine - 76800;
  h = (((((adc_H << 14) - ((int32_t)dig_H4 << 20) - ((int32_t)dig_H5 * h)) + 16384) >> 15) * 
       (((((((h * (int32_t)dig_H6) >> 10) * (((h * (int32_t)dig_H3) >> 11) + 32768)) >> 10) + 2097152) * 
       (int32_t)dig_H2 + 8192) >> 14));
  h = h - (((((h >> 15) * (h >> 15)) >> 7) * (int32_t)dig_H1) >> 4);
  h = h < 0 ? 0 : h;
  h = h > 419430400 ? 419430400 : h;
  *humidity = (float)h / 1024.0f;

// 高度の計算
  // h = 44330 * (1 - (p/p0)^(1/5.255))
  *altitude = PRESSURE_SCALING * (1.0f - pow(*pressure / SEALEVEL_PRESSURE, 0.190295f));
}

// ヘルパー関数
uint8_t read8(uint8_t reg) {
  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(BME280_ADDRESS, 1);
  return Wire.read();
}

uint16_t read16_LE(uint8_t reg) {
  uint16_t temp = read16(reg);
  return (temp >> 8) | (temp << 8);
}

int16_t readS16_LE(uint8_t reg) {
  return (int16_t)read16_LE(reg);
}

uint16_t read16(uint8_t reg) {
  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(BME280_ADDRESS, 2);
  return (Wire.read() << 8) | Wire.read();
}

void write8(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void readRegister(uint8_t reg, uint8_t *data, uint8_t len) {
  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom((uint16_t)BME280_ADDRESS, (uint8_t)len);
  for (uint8_t i = 0; i < len; i++) {
    data[i] = Wire.read();
  }
}