# 機体情報を管理
# パケットを作成
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

    def parse_data(self, data):
        # print(f"Data: {data} data[0]: {data[0]}")
        data_len = len(data)
        if (data_len == 54) and data[0] == 0x55:
            self.seqno = data[1]
            # サーボ値の解析
            for i in range(4):
                self.servo[i] = (data[2+i*2] << 8) + data[3+i*2]
            
            # PID値の解析
            pid_roll_sign = data[10]
            pid_roll = (data[11] << 8) + data[12]
            if pid_roll_sign:
                pid_roll = -pid_roll

            pid_pitch_sign = data[13]
            pid_pitch = (data[14] << 8) + data[15]
            if pid_pitch_sign:
                pid_pitch = -pid_pitch
            
            pid_yaw_sign = data[16]
            pid_yaw = (data[17] << 8) + data[18]
            if pid_yaw_sign:
                pid_yaw = -pid_yaw

            self.pid['roll'] = pid_roll / 100
            self.pid['pitch'] = pid_pitch / 100
            self.pid['yaw'] = pid_yaw / 100

            # IMU角度の解析
            imu_roll_sign = data[19]
            imu_roll = (data[20] << 8) + data[21]
            if imu_roll_sign:
                imu_roll = -imu_roll
            
            imu_pitch_sign = data[22]
            imu_pitch = (data[23] << 8) + data[24]
            if imu_pitch_sign:
                imu_pitch = -imu_pitch
            
            imu_yaw_sign = data[25]
            imu_yaw = (data[26] << 8) + data[27]
            if imu_yaw_sign:
                imu_yaw = -imu_yaw
            # print(f"imu_roll: {imu_roll}, imu_pitch: {imu_pitch}, imu_yaw: {imu_yaw}")
            
            self.imu['roll'] = imu_roll / 100
            self.imu['pitch'] = imu_pitch / 100
            self.imu['yaw'] = imu_yaw / 100
            
            # ジャイロ値の解析
            gyro_x_sign = data[28]
            gyro_x = (data[29] << 8) + data[30]
            if gyro_x_sign:
                gyro_x = -gyro_x

            gyro_y_sign = data[31]
            gyro_y = (data[32] << 8) + data[33]
            if gyro_y_sign:
                gyro_y = -gyro_y

            gyro_z_sign = data[34]
            gyro_z = (data[35] << 8) + data[36]
            if gyro_z_sign:
                gyro_z = -gyro_z

            self.gyro['x'] = gyro_x / 100
            self.gyro['y'] = gyro_y / 100
            self.gyro['z'] = gyro_z / 100
            
            # バッテリー電圧の解析
            battery_vol = (data[37] << 8) + data[38]
            self.battery = battery_vol / 1000
            
            # 高度の解析
            altitude = (data[39] << 8) + data[40]
            if altitude & 0x8000:  # 符号ビットが1の場合
              altitude -= 0x10000
            self.altitude = altitude / 1000

            # 温度の解析
            temperature = (data[41] << 8) + data[42]
            self.temperature = temperature / 1000

            # キャリブレーションリクエスト
            calibrate =  (data[43] << 8) + data[44]
            if calibrate:
                self.calibrate = True

            # 加速度値の解析
            acc_x_sign = data[45]
            acc_x = (data[46] << 8) + data[47]
            if acc_x_sign:
                acc_x = -acc_x
            self.acc['x'] = acc_x / 100

            acc_y_sign = data[48]
            acc_y = (data[49] << 8) + data[50]  
            if acc_y_sign:
                acc_y = -acc_y
            self.acc['y'] = acc_y / 100

            acc_z_sign = data[51]
            acc_z = (data[52] << 8) + data[53]  
            if acc_z_sign:
                acc_z = -acc_z
            self.acc['z'] = acc_z / 100

            # IMUとPIDとの差分
            self.ip['roll'] = self.imu['roll'] + self.pid['roll']
            self.ip['pitch'] = self.imu['pitch'] + self.pid['pitch']
            self.ip['yaw'] = self.imu['yaw'] + self.pid['yaw']

            return True
        return False