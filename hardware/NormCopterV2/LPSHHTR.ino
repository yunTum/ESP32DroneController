#include <Wire.h>

class LPS22HH {
private:
    // センサーのアドレス
    static const uint8_t LPS22HH_ADDRESS = 0x5D;
    // レジスタアドレス
    static const uint8_t WHO_AM_I     = 0x0F;
    static const uint8_t CTRL_REG1    = 0x10;
    static const uint8_t CTRL_REG2    = 0x11;
    static const uint8_t CTRL_REG3    = 0x12;
    static const uint8_t IF_CTRL      = 0x0E;
    static const uint8_t FIFO_CTRL    = 0x14;    // FIFOコントロール
    static const uint8_t BDU          = 0x02;    // Block Data Update bit in CTRL_REG1
    static const uint8_t LOW_NOISE_EN = 0x02;    // Low Noise Enable bit in CTRL_REG1
    static const uint8_t FILTER_CFG   = 0x13;    // フィルタ設定レジスタ
    static const uint8_t PRESS_OUT_XL = 0x28;
    static const uint8_t PRESS_OUT_L  = 0x29;
    static const uint8_t PRESS_OUT_H  = 0x2A;
    static const uint8_t TEMP_OUT_L   = 0x2B;
    static const uint8_t TEMP_OUT_H   = 0x2C;
    static const uint8_t INTERRUPT_CFG = 0x0B;    // 割り込み設定
    static const uint8_t THS_P_L       = 0x0C;    // 気圧閾値 下位バイト
    static const uint8_t THS_P_H       = 0x0D;    // 気圧閾値 上位バイト
    static const uint8_t FIFO_WTM      = 0x15;    // FIFOウォーターマーク
    static const uint8_t REF_P_XL      = 0x15;    // リファレンス気圧 XL
    static const uint8_t REF_P_L       = 0x16;    // リファレンス気圧 L
    static const uint8_t REF_P_H       = 0x17;    // リファレンス気圧 H
    
    float reference_pressure; // 基準気圧
    bool is_calibrated; // キャリブレーション状態
    TwoWire *wire;
    uint8_t addr;
    
    uint8_t readByte(uint8_t reg) {
        wire->beginTransmission(addr);
        wire->write(reg);
        wire->endTransmission(false);
        wire->requestFrom(addr, (uint8_t)1);
        return wire->read();
    }
    
    void writeByte(uint8_t reg, uint8_t data) {
        wire->beginTransmission(addr);
        wire->write(reg);
        wire->write(data);
        wire->endTransmission();
    }

public:
    float pressure;    // hPa
    float temperature; // ℃
    float altitude;    // m
    
    LPS22HH(TwoWire &w, uint8_t address = LPS22HH_ADDRESS) : wire(&w), addr(address) {
        pressure = 0.0;
        temperature = 0.0;
        altitude = 0.0;
        reference_pressure = 1013.25;
        is_calibrated = false;
    }
    
    bool begin() {
        // センサーの存在確認
        if (readByte(WHO_AM_I) != 0xB3) {
            return false;
        }
        
        // FIFOモードの設定（連続モード）
        writeByte(FIFO_CTRL, 0xC0);  // Continuous mode
        writeByte(FIFO_WTM, 0x0F);   // 16サンプルの平均を取る
        
        // 自動ゼロ機能の設定
        // INTERRUPT_CFGでAUTOZERO=1を設定
        writeByte(INTERRUPT_CFG, 0x10);
        
        // センサーの初期設定
        // CTRL_REG1: ODR=75Hz(0x50), ODR=200Hz(0x70),LOW_NOISE_EN=1, BDU=1
        // writeByte(CTRL_REG1, 0x70 | LOW_NOISE_EN | BDU);  
        writeByte(CTRL_REG1, 0x50);  // 75Hz出力、LPFなし
        writeByte(CTRL_REG2, 0x10);  // 連続測定モード

        // フィルタ設定
        // ローパスフィルタを有効化し、最大のフィルタリング効果を設定
        writeByte(FILTER_CFG, 0x0C);  // ODR/20フィルタリング
        return true;
    }

    // キャリブレーション実行
    bool calibrate() {
        const int SAMPLES = 10;  // 平均を取るサンプル数
        float sum = 0.0;
        
        // 複数回測定して平均を取る
        for (int i = 0; i < SAMPLES; i++) {
            readData();
            sum += pressure;
            delay(100);  // センサーの安定を待つ
        }
        
        reference_pressure = sum / SAMPLES;
        is_calibrated = true;
        altitude = 0.0;  // キャリブレーション位置を0mとする
        
        return true;
    }

    // キャリブレーション状態の確認
    bool isCalibrated() {
        return is_calibrated;
    }
    
    // 基準気圧を設定
    void setReferencePressure(float ref_pressure) {
        reference_pressure = ref_pressure;
    }

    // 現在の気圧を基準気圧として設定（その場所をゼロ点とする）
    void calibrateSeaLevel() {
        reference_pressure = pressure;
    }

    float getReferencePressure() {
        return reference_pressure;
    }

    void readData() {
        // 気圧データの読み取り (3バイト)
        uint32_t press_raw = readByte(PRESS_OUT_XL);
        press_raw |= (uint32_t)readByte(PRESS_OUT_L) << 8;
        press_raw |= (uint32_t)readByte(PRESS_OUT_H) << 16;
        pressure = press_raw / 4096.0;  // LSB / 4096 = hPa
        
        // 温度データの読み取り (2バイト)
        int16_t temp_raw = readByte(TEMP_OUT_L);
        temp_raw |= (int16_t)readByte(TEMP_OUT_H) << 8;
        temperature = temp_raw / 100.0;  // LSB / 100 = ℃

        // 高度の計算（国際標準大気モデル）
        // h = 44330 * (1 - (p/p0)^(1/5.255))
        if (is_calibrated) {
            // 温度補正を含めた高度計算
            const float R = 287.052;      // 乾燥空気の気体定数 [J/(kg·K)]
            const float g = 9.80665;      // 重力加速度 [m/s^2]
            float T_kelvin = temperature + 273.15;  // 絶対温度 [K]
            
            altitude = (T_kelvin * R / g) * log(reference_pressure / pressure);
        }
    }

    // FIFOからの平均値を取得する関数
    void readAveragedData() {
        float press_sum = 0;
        float temp_sum = 0;
        const int samples = 16;  // FIFOサイズ

        for (int i = 0; i < samples; i++) {
            uint32_t press_raw = readByte(PRESS_OUT_XL);
            press_raw |= (uint32_t)readByte(PRESS_OUT_L) << 8;
            press_raw |= (uint32_t)readByte(PRESS_OUT_H) << 16;
            press_sum += press_raw / 4096.0;

            int16_t temp_raw = readByte(TEMP_OUT_L);
            temp_raw |= (int16_t)readByte(TEMP_OUT_H) << 8;
            temp_sum += temp_raw / 100.0;
        }

        pressure = press_sum / samples;
        temperature = temp_sum / samples;

        // 高度計算（既存のコード）
        if (is_calibrated) {
            const float R = 287.052;
            const float g = 9.80665;
            float T_kelvin = temperature + 273.15;
            altitude = (T_kelvin * R / g) * log(reference_pressure / pressure);
        }
    }
};

// グローバルインスタンス
LPS22HH lps(Wire);

float pressure, temperature, altitude;
void LPSHHTR_init() {
    
    if (!lps.begin()) {
        Serial.println("LPS22HH ID NG");
        while (1) delay(10);
    }
    Serial.println("LPS22HH ID OK");

    // キャリブレーション実行
    if (lps.calibrate()) {
        Serial.println("Pressure Calibrated");
        lps.calibrateSeaLevel();
        Serial.print("Reference Pressure: ");
        Serial.print(lps.getReferencePressure());
        Serial.println(" hPa");
    } else {
        Serial.println("Calibration Failed");
    }
}

void getLPS22HH() {
    lps.readData();
    if (lps.isCalibrated()) {
        // 気圧 hPa
        pressure = lps.pressure;
        // 温度 ℃
        temperature = lps.temperature;
        // 高度 m
        altitude = lps.altitude;
    }
}