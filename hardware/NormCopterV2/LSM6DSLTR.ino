#include <Wire.h>
#include <EEPROM.h>

#define LSM6DSLTR_ADDRESS 0x6B
#define CALSTEPS 512  // キャリブレーションのサンプル数
#define ACCRESO 4096  // 加速度センサーの分解能

// サンプリング周波数の設定
#define ODR_XL_26Hz    0x30  // 加速度センサー: 26Hz
#define ODR_XL_208Hz   0x4C  // 加速度センサー: 208Hz
#define ODR_XL_416Hz   0x5C  // 加速度センサー: 416Hz

#define ODR_G_26Hz     0x30  // ジャイロセンサー: 26Hz
#define ODR_G_208Hz    0x4C  // ジャイロセンサー: 208Hz
#define ODR_G_416Hz    0x5C  // ジャイロセンサー: 416Hz

// 軸の定義
#define ROLL     0
#define PITCH    1
#define YAW      2

// フィルタリング用の定数
#define FILTER_SIZE 10  // 移動平均フィルタのサイズ
#define ALPHA 0.96f     // 相補フィルタの係数

int16_t gyroADC[3];
int16_t accADC[3];
int16_t gyroZero[3] = {0,0,0};
int16_t accZero[3] = {0,0,0};
int calibratingG = CALSTEPS;
int calibratingA = CALSTEPS;

// フィルタリング用の変数
float filtered_acc[3] = {0};
float filtered_gyro[3] = {0};
float acc_history[3][FILTER_SIZE] = {0};
float gyro_history[3][FILTER_SIZE] = {0};
int filter_index = 0;

// センサーの向きを定義
// pitch：上向きが正
// roll：右向きが正
// yaw：時計回りが正
#define ACC_ORIENTATION(X, Y, Z)  {accADC[PITCH] = -X; accADC[ROLL] = Y; accADC[YAW] = -Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = Y; gyroADC[PITCH] = -X; gyroADC[YAW] = Z;}

// レジスタアドレス
#define WHO_AM_I      0x0F
#define CTRL1_XL      0x10
#define CTRL2_G       0x11
#define CTRL3_C       0x12
#define CTRL4_C       0x13
#define CTRL8_XL      0x17
#define OUTX_L_G      0x22
#define OUTX_H_G      0x23
#define OUTY_L_G      0x24
#define OUTY_H_G      0x25
#define OUTZ_L_G      0x26
#define OUTZ_H_G      0x27
#define OUTX_L_XL     0x28
#define OUTX_H_XL     0x29
#define OUTY_L_XL     0x2A
#define OUTY_H_XL     0x2B
#define OUTZ_L_XL     0x2C
#define OUTZ_H_XL     0x2D

// ジャイロスケール設定
#define GYRO_SCALE_250  0x00  // ±250 dps
#define GYRO_SCALE_500  0x04  // ±500 dps
#define GYRO_SCALE_1000 0x08  // ±1000 dps
#define GYRO_SCALE_2000 0x0C  // ±2000 dps

// 加速度スケール設定
#define ACC_SCALE_2G   0x00  // ±2g
#define ACC_SCALE_4G   0x08  // ±4g
#define ACC_SCALE_8G   0x0C  // ±8g
#define ACC_SCALE_16G  0x04  // ±16g

#define GYRO_SCALE GYRO_SCALE_250
#define ACC_SCALE ACC_SCALE_2G

// ジャイロスケール変換係数
#if GYRO_SCALE == GYRO_SCALE_250
    #define GYRO_SCALE_FACTOR 8.750f  // mdps/LSB
#elif GYRO_SCALE == GYRO_SCALE_500
    #define GYRO_SCALE_FACTOR 17.50f  // mdps/LSB
#elif GYRO_SCALE == GYRO_SCALE_1000
    #define GYRO_SCALE_FACTOR 35.00f  // mdps/LSB
#elif GYRO_SCALE == GYRO_SCALE_2000
    #define GYRO_SCALE_FACTOR 70.00f  // mdps/LSB
#endif

// 加速度スケール変換係数
#if ACC_SCALE == ACC_SCALE_2G
    #define ACC_SCALE_FACTOR 0.061f  // mg/LSB
#elif ACC_SCALE == ACC_SCALE_4G
    #define ACC_SCALE_FACTOR 0.122f  // mg/LSB
#elif ACC_SCALE == ACC_SCALE_8G
    #define ACC_SCALE_FACTOR 0.244f  // mg/LSB
#elif ACC_SCALE == ACC_SCALE_16G
    #define ACC_SCALE_FACTOR 0.488f  // mg/LSB
#endif

// ジャイロ取得周波数
#if defined(ODR_G_416Hz)
    #define GYRO_DT 0.002403846f  // 1/416Hz
#elif defined(ODR_G_208Hz)
    #define GYRO_DT 0.004807692f  // 1/208Hz
#elif defined(ODR_G_26Hz)
    #define GYRO_DT 0.038461538f  // 1/26Hz
#endif

// 加速度取得周波数
#if defined(ODR_XL_416Hz)
    #define ACC_DT 0.002403846f  // 1/416Hz
#elif defined(ODR_XL_208Hz)
    #define ACC_DT 0.004807692f  // 1/208Hz
#elif defined(ODR_XL_26Hz)
    #define ACC_DT 0.038461538f  // 1/26Hz
#endif

void i2cRead(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data) {
    Wire.beginTransmission(Address);
    Wire.write(Register);
    Wire.endTransmission(false);
    Wire.requestFrom(Address, Nbytes);
    uint8_t index = 0;
    while (Wire.available())
        Data[index++] = Wire.read();
}

void i2cWriteByte(uint8_t Address, uint8_t Register, uint8_t Data) {
    Wire.beginTransmission(Address);
    Wire.write(Register);
    Wire.write(Data);
    Wire.endTransmission();
}

// 移動平均フィルタの実装
void applyMovingAverage(float value, float history[], float &filtered) {
    history[filter_index] = value;
    float sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
        sum += history[i];
    }
    filtered = sum / FILTER_SIZE;
}

// 相補フィルタの実装
void applyComplementaryFilter(float acc_value, float gyro_value, float &filtered, float dt) {
    filtered = ALPHA * (filtered + gyro_value * dt) + (1.0f - ALPHA) * acc_value;
}

void GYRO_Common() {
    static int32_t g[3];
    uint8_t axis;

    if (calibratingG > 0) {
        for (axis = 0; axis < 3; axis++) {
            if (calibratingG == CALSTEPS) g[axis] = 0;
            g[axis] += gyroADC[axis];
            gyroADC[axis] = 0;
            gyroZero[axis] = 0;
            if (calibratingG == 1) {
                if (g[axis] >= 0) g[axis] += CALSTEPS/2;
                else              g[axis] -= CALSTEPS/2;
                gyroZero[axis] = g[axis]/CALSTEPS;
            }
        }
        calibratingG--;
        if (calibratingG == 0) {
            Serial.println("Gyro Calibrated:");
            Serial.printf("X: %d  Y: %d  Z: %d\n", gyroZero[0], gyroZero[1], gyroZero[2]);
        }
    }

    for (axis = 0; axis < 3; axis++) {
        gyroADC[axis] = gyroADC[axis] - gyroZero[axis];
        // 移動平均フィルタを適用
        // applyMovingAverage(gyroADC[axis], gyro_history[axis], filtered_gyro[axis]);
        // PID制御用の変数に代入
        if (axis == ROLL) GyroX = gyroADC[axis];
        if (axis == PITCH) GyroY = gyroADC[axis];
        if (axis == YAW) GyroZ = gyroADC[axis];
    }
}

void readacc() {
    accZero[0] = EEPROM.readShort(0);
    accZero[1] = EEPROM.readShort(2);
    accZero[2] = EEPROM.readShort(6);
}

void storeacc() {
    EEPROM.writeShort(0, accZero[0]);
    EEPROM.writeShort(2, accZero[1]);
    EEPROM.writeShort(6, accZero[2]);
    EEPROM.commit();
}

void ACC_Common() {
    static int32_t a[3];
    // static unsigned long last_time = 0;
    // float dt = ACC_DT;
    // last_time = micros();

    if (calibratingA > 0) {
        for (uint8_t axis = 0; axis < 3; axis++) {
            if (calibratingA == CALSTEPS) a[axis] = 0;
            a[axis] += accADC[axis];
            accADC[axis] = 0;
            accZero[axis] = 0;
        }
        if (calibratingA == 1) {
            // for (uint8_t axis = 0; axis < 3; axis++) {
            //     if (a[axis] >= 0) a[axis] += CALSTEPS/2;
            //     else             a[axis] -= CALSTEPS/2;
            //     accZero[axis] = a[axis]/CALSTEPS;
            // }
            // accZero[2] -= ACCRESO;
            if (a[0] >= 0) a[0] += CALSTEPS/2; else a[0] -= CALSTEPS/2;
            if (a[1] >= 0) a[1] += CALSTEPS/2; else a[1] -= CALSTEPS/2;
            if (a[2] >= 0) a[2] += CALSTEPS/2; else a[2] -= CALSTEPS/2;
            accZero[0] = a[0]/CALSTEPS;
            accZero[1] = a[1]/CALSTEPS;
            accZero[2] = (a[2]/CALSTEPS) - ACCRESO;
            storeacc();
        }
        calibratingA--;
        if (calibratingA == 0) {
            Serial.println("Acc Calibrated:");
            Serial.printf("X: %d  Y: %d  Z: %d\n", accZero[0], accZero[1], accZero[2]);
        }
    }
    
    for (uint8_t axis = 0; axis < 3; axis++) {
        accADC[axis] -= accZero[axis];
        // // 移動平均フィルタと相補フィルタを適用
        // applyMovingAverage(accADC[axis], acc_history[axis], filtered_acc[axis]);
        // applyComplementaryFilter(filtered_acc[axis], filtered_gyro[axis], filtered_acc[axis], dt);
    
        // PID制御用の変数に代入
        if (axis == ROLL) AccX = float(accADC[axis]) * 0.01;
        if (axis == PITCH) AccY = float(accADC[axis]) * 0.01;
        if (axis == YAW) AccZ = float(accADC[axis]) * 0.01;
    }
    // filter_index = (filter_index + 1) % FILTER_SIZE;
}

uint8_t rawADC[12];

void GyroAcc_getADC() {
    uint8_t rawADC[12];
    
    // バースト読み取りで全データを一度に取得
    i2cRead(LSM6DSLTR_ADDRESS, OUTX_L_G, 12, rawADC);
    
    // ジャイロデータの処理
    int16_t gyro_x = (int16_t)((rawADC[1] << 8) | rawADC[0]);
    int16_t gyro_y = (int16_t)((rawADC[3] << 8) | rawADC[2]);
    int16_t gyro_z = (int16_t)((rawADC[5] << 8) | rawADC[4]);

    // スケール変換を適用（mdps → dps）
    float gyro_x_dps = gyro_x * GYRO_SCALE_FACTOR / 100.0f;
    float gyro_y_dps = gyro_y * GYRO_SCALE_FACTOR / 100.0f;
    float gyro_z_dps = gyro_z * GYRO_SCALE_FACTOR / 100.0f;

    GYRO_ORIENTATION(gyro_x_dps, gyro_y_dps, gyro_z_dps);
    GYRO_Common();
    
    // 加速度データの処理
    int16_t acc_x = (int16_t)((rawADC[7] << 8) | rawADC[6]);
    int16_t acc_y = (int16_t)((rawADC[9] << 8) | rawADC[8]);
    int16_t acc_z = (int16_t)((rawADC[11] << 8) | rawADC[10]);

    // スケール変換を適用（mG → G）
    float acc_x_g = acc_x * ACC_SCALE_FACTOR * 32768 / 1000.0f;
    float acc_y_g = acc_y * ACC_SCALE_FACTOR * 32768 / 1000.0f;
    float acc_z_g = acc_z * ACC_SCALE_FACTOR * 32768 / 1000.0f;

    // 生の16ビット値をそのまま使用
    ACC_ORIENTATION(acc_x_g, acc_y_g, acc_z_g);
    ACC_Common();
}

void LSM6DSLTR_init() {
    // センサーの存在確認
    uint8_t id;
    i2cRead(LSM6DSLTR_ADDRESS, WHO_AM_I, 1, &id);
    if (id == 0x6A) Serial.println("LSM6DSLTR ID OK");
    else Serial.println("LSM6DSLTR ID Failed");

    // センサーの高精度設定
    i2cWriteByte(LSM6DSLTR_ADDRESS, CTRL1_XL, ODR_XL_416Hz | ACC_SCALE);  // 加速度: ±4g, 416Hz, アンチエイリアシングフィルタ
    i2cWriteByte(LSM6DSLTR_ADDRESS, CTRL2_G, ODR_G_416Hz | GYRO_SCALE);   // ジャイロ: ±500dps, 416Hz
    // i2cWriteByte(LSM6DSLTR_ADDRESS, CTRL1_XL, ODR_XL_208Hz | ACC_SCALE);  // 加速度: 208Hz
    // i2cWriteByte(LSM6DSLTR_ADDRESS, CTRL2_G, ODR_G_208Hz | GYRO_SCALE);  // ジャイロ: 208Hz
    // i2cWriteByte(LSM6DSLTR_ADDRESS, CTRL1_XL, ODR_XL_26Hz | ACC_SCALE);
    // i2cWriteByte(LSM6DSLTR_ADDRESS, CTRL2_G, ODR_G_26Hz | GYRO_SCALE);
    i2cWriteByte(LSM6DSLTR_ADDRESS, CTRL3_C, 0x44);   // BDU有効, アドレス自動インクリメント
    i2cWriteByte(LSM6DSLTR_ADDRESS, CTRL4_C, 0x02);   // ジャイロLPF1有効
    i2cWriteByte(LSM6DSLTR_ADDRESS, CTRL8_XL, 0x89);  // LPF2有効(0x80), ODR/4ローパスフィルタリング(0x08), 6D機能LPF2有効(0x01)
    // i2cWriteByte(LSM6DSLTR_ADDRESS, CTRL8_XL, 0x0B);
    delay(150);
    readacc();  // EEPROMから加速度オフセットを読み込み
}

void calibrate() {
    // キャリブレーションの開始
    calibratingG = CALSTEPS;
    calibratingA = CALSTEPS;
    
    // キャリブレーションが完了するまで待機
    while (calibratingG > 0 || calibratingA > 0) {
        GyroAcc_getADC();
        delay(1);
    }
    
    // キャリブレーション完了後、結果を送信
    Serial.println("Calibration Complete");
    Serial.printf("Gyro Offset: X: %d  Y: %d  Z: %d\n", gyroZero[0], gyroZero[1], gyroZero[2]);
    Serial.printf("Acc Offset: X: %d  Y: %d  Z: %d\n", accZero[0], accZero[1], accZero[2]);

    // クライアントにキャリブレーション完了を通知
    calibrateRequest = false;  // キャリブレーションリクエストをfalseに設定
}
