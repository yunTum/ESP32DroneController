# 機体情報を管理
# パケットを作成
import time

class RemoteData():
  def __init__(self):
    self.chnum = 8
    self.rc_value = [1515] * self.chnum
    self.rc_value[0] = 50 #Range 50 - 1800
    self.seqno = 0
    
  def store_RC(self, in_value, out, offset):
    out[offset] = (in_value >> 8) & 0xFF  # 上位バイト
    out[offset + 1] = in_value & 0xFF     # 下位バイト
  
  def create_byte(self):
    buff = bytearray(18)
    buff[0] = 0x55                            # チェック値
    buff[1] = self.seqno                      # シーケンス番号
    self.store_RC(self.rc_value[1], buff,  2) # ROLL
    self.store_RC(self.rc_value[2], buff,  4) # PITCH
    self.store_RC(self.rc_value[0], buff,  6) # THROTTLE
    self.store_RC(self.rc_value[3], buff,  8) # RUDDER
    self.store_RC(self.rc_value[4], buff, 10) # ARM
    self.store_RC(self.rc_value[5], buff, 12) # RADIO6
    self.store_RC(self.rc_value[6], buff, 14) # RADIO7
    self.store_RC(self.rc_value[7], buff, 16) # CALIBRATE
    return buff
  
  def defalut_set(self):
    self.rc_value = [1515] * self.chnum
    self.rc_value[0] = 50

class DroneData():
    def __init__(self):
        self.servo = [0] * 4
        self.pid = {'roll': 0, 'pitch': 0, 'yaw': 0}
        self.imu = {'roll': 0, 'pitch': 0, 'yaw': 0}
        self.gyro = {'x': 0, 'y': 0, 'z': 0}
        self.battery = 0
        self.seqno = 0
        self.altitude = 0
        self.temperature = 0
        self.acc = {'x': 0, 'y': 0, 'z': 0}
        # IMUとPIDとの差分
        self.ip = {'roll': 0, 'pitch': 0, 'yaw': 0}
        self.calibrate = False

        self.last_velocity = {'x': 0, 'y': 0, 'z': 0}
        self.last_time = time.time()
        self.calculated_altitude = 0
        self.last_att = 0

    def parse_data(self, data):
        # print(f"Data: {data} data[0]: {data[0]}")
        data_len = len(data)
        if (data_len == 60) and data[0] == 0x55:
            self.seqno = (data[1] << 8) + data[2]
            # サーボ値の解析
            for i in range(4):
                self.servo[i] = (data[3+i*2] << 8) + data[4+i*2]
            
            # PID値の解析
            pid_roll_sign = data[11]
            pid_roll = (data[12] << 8) + data[13]
            if pid_roll_sign:
                pid_roll = -pid_roll

            pid_pitch_sign = data[14]
            pid_pitch = (data[15] << 8) + data[16]
            if pid_pitch_sign:
                pid_pitch = -pid_pitch
            
            pid_yaw_sign = data[17]
            pid_yaw = (data[18] << 8) + data[19]
            if pid_yaw_sign:
                pid_yaw = -pid_yaw

            self.pid['roll'] = pid_roll / 100
            self.pid['pitch'] = pid_pitch / 100
            self.pid['yaw'] = pid_yaw / 100

            # IMU角度の解析
            imu_roll_sign = data[20]
            imu_roll = (data[21] << 8) + data[22]
            if imu_roll_sign:
                imu_roll = -imu_roll
            
            imu_pitch_sign = data[23]
            imu_pitch = (data[24] << 8) + data[25]
            if imu_pitch_sign:
                imu_pitch = -imu_pitch

            imu_yaw_sign = data[26]
            imu_yaw = (data[27] << 8) + data[28]
            if imu_yaw_sign:
                imu_yaw = -imu_yaw
            # print(f"imu_roll: {imu_roll}, imu_pitch: {imu_pitch}, imu_yaw: {imu_yaw}")
            
            self.imu['roll'] = imu_roll / 100
            self.imu['pitch'] = imu_pitch / 100
            self.imu['yaw'] = imu_yaw / 100
            
            # ジャイロ値の解析
            gyro_x_sign = data[29]
            gyro_x = (data[30] << 8) + data[31]
            if gyro_x_sign:
                gyro_x = -gyro_x

            gyro_y_sign = data[32]
            gyro_y = (data[33] << 8) + data[34]
            if gyro_y_sign:
                gyro_y = -gyro_y

            gyro_z_sign = data[35]
            gyro_z = (data[36] << 8) + data[37]
            if gyro_z_sign:
                gyro_z = -gyro_z

            self.gyro['x'] = gyro_x / 100
            self.gyro['y'] = gyro_y / 100
            self.gyro['z'] = gyro_z / 100
            
            # バッテリー電圧の解析
            battery_vol = (data[38] << 8) + data[39]
            self.battery = battery_vol / 1000
            
            # 高度の解析
            altitude = (data[40] << 8) + data[41]
            if altitude & 0x8000:  # 符号ビットが1の場合
              altitude -= 0x10000
            self.altitude = altitude / 1000

            # 温度の解析
            temperature = (data[42] << 8) + data[43]
            self.temperature = temperature / 1000

            # キャリブレーションリクエスト
            calibrate =  (data[44] << 8) + data[45]
            if calibrate:
                self.calibrate = True

            # 加速度値の解析
            acc_x_sign = data[46]
            acc_x = (data[47] << 8) + data[48]
            if acc_x_sign:
                acc_x = -acc_x
            self.acc['x'] = acc_x / 100

            acc_y_sign = data[49]
            acc_y = (data[50] << 8) + data[51]  
            if acc_y_sign:
                acc_y = -acc_y
            self.acc['y'] = acc_y / 100

            acc_z_sign = data[52]
            acc_z = (data[53] << 8) + data[54]  
            if acc_z_sign:
                acc_z = -acc_z
            self.acc['z'] = acc_z / 100

            # シンクタイムの解析 8バイト msec
            sync_time = (data[58] << 24) + (data[57] << 16) + (data[56] << 8) + data[55]
            self.sync_time = sync_time / 1000 # sec

            self.calculate_altitude_from_acc()

            # IMUとPIDとの差分
            self.ip['roll'] = self.imu['roll'] + self.pid['roll']
            self.ip['pitch'] = self.imu['pitch'] + self.pid['pitch']
            self.ip['yaw'] = self.imu['yaw'] + self.pid['yaw']

            log_data = [
                f"{time.strftime('%Y-%m-%d %H:%M:%S')}",  # タイムスタンプ
                f"{self.seqno}",                          # シーケンス番号
                f"{self.sync_time}",                      # シンクタイム
                f"{','.join(map(str, self.servo))}",      # サーボ値
                f"{self.pid['roll']:.2f}",                # PID Roll
                f"{self.pid['pitch']:.2f}",               # PID Pitch
                f"{self.pid['yaw']:.2f}",                 # PID Yaw
                f"{self.imu['roll']:.2f}",                # IMU Roll
                f"{self.imu['pitch']:.2f}",               # IMU Pitch
                f"{self.imu['yaw']:.2f}",                 # IMU Yaw
                f"{self.gyro['x']:.2f}",                  # ジャイロX
                f"{self.gyro['y']:.2f}",                  # ジャイロY
                f"{self.gyro['z']:.2f}",                  # ジャイロZ
                f"{self.acc['x']:.2f}",                   # 加速度X
                f"{self.acc['y']:.2f}",                   # 加速度Y
                f"{self.acc['z']:.2f}",                   # 加速度Z
                f"{self.battery:.2f}",                    # バッテリー電圧
                f"{self.altitude:.2f}",                   # 高度
                f"{self.temperature:.2f}",                # 温度
                f"{self.calculated_altitude:.2f}"         # 算出高度
            ]
            
            # CSVファイルに書き込み
            with open('./flight_log/log.csv', 'a', encoding='utf-8') as f:
                f.write(','.join(log_data) + '\n')

            return True
            # エラー時のログ
        with open('./flight_log/log.csv', 'a', encoding='utf-8') as f:
            f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')},ERROR,データ解析エラー\n")
        return False
    
    def calculate_altitude_from_acc(self):
        current_time = self.sync_time
        if self.seqno == 0:
            self.last_time = current_time
            self.last_velocity = {'x': 0, 'y': 0, 'z': 0}
            self.calculated_altitude = 0
        dt = current_time - self.last_time
        
        # 重力加速度の補正（9.81 m/s^2）
        adjusted_acc_z = self.acc['z'] - 41.0
        
        # 速度の計算（加速度の積分）
        new_velocity_z = self.last_velocity['z'] + adjusted_acc_z * dt
        
        # 位置（高度）の計算（速度の積分）
        self.calculated_altitude += (new_velocity_z + self.last_velocity['z']) * dt / 2
        
        # 値の更新
        self.last_velocity['z'] = new_velocity_z
        self.last_time = current_time