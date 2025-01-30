# 機体情報を管理
# パケットを作成
class RemoteData():
  def __init__(self):
    self.chnum = 7
    self.rc_value = [1515] * self.chnum
    self.rc_value[0] = 50 #Range 50 - 1800
    self.seqno = 0
    
  def store_RC(self, in_value, out, offset):
    out[offset] = (in_value >> 8) & 0xFF  # 上位バイト
    out[offset + 1] = in_value & 0xFF     # 下位バイト
  
  def create_byte(self):
    buff = bytearray(16)
    buff[0] = 0x55                            # チェック値
    buff[1] = self.seqno                      # シーケンス番号
    self.store_RC(self.rc_value[1], buff,  2) # ROLL
    self.store_RC(self.rc_value[2], buff,  4) # PITCH
    self.store_RC(self.rc_value[0], buff,  6) # THROTTLE
    self.store_RC(self.rc_value[3], buff,  8) # RUDDER
    self.store_RC(self.rc_value[4], buff, 10) # ARM
    self.store_RC(self.rc_value[5], buff, 12) # RADIO6
    self.store_RC(self.rc_value[6], buff, 14) # RADIO7
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
    
    def parse_data(self, data):
        # print(f"Data: {data} data[0]: {data[0]}")
        data_len = len(data)
        if (data_len == 30) or (data_len == 34) and data[0] == 0x55:
            self.seqno = data[1]
            # サーボ値の解析
            for i in range(4):
                self.servo[i] = (data[2+i*2] << 8) + data[3+i*2]
            
            # PID値の解析
            pid_roll = (data[10] << 8) + data[11]
            if pid_roll & 0x8000:  # 符号ビットが1の場合
                pid_roll -= 0x10000
            
            pid_pitch = (data[12] << 8) + data[13]
            if pid_pitch & 0x8000:  # 符号ビットが1の場合
                pid_pitch -= 0x10000
            
            pid_yaw = (data[14] << 8) + data[15]
            if pid_yaw & 0x8000:  # 符号ビットが1の場合
                pid_yaw -= 0x10000
            
            self.pid['roll'] = pid_roll / 1000
            self.pid['pitch'] = pid_pitch / 1000
            self.pid['yaw'] = pid_yaw / 1000

            # IMU角度の解析
            imu_roll = (data[16] << 8) + data[17]
            if imu_roll & 0x8000:  # 符号ビットが1の場合
                imu_roll -= 0x10000

            imu_pitch = (data[18] << 8) + data[19]
            if imu_pitch & 0x8000:  # 符号ビットが1の場合
                imu_pitch -= 0x10000
            
            imu_yaw = (data[20] << 8) + data[21]
            if imu_yaw & 0x8000:  # 符号ビットが1の場合
                imu_yaw -= 0x10000
            
            self.imu['roll'] = imu_roll / 1000
            self.imu['pitch'] = imu_pitch / 1000
            self.imu['yaw'] = imu_yaw / 1000
            
            # ジャイロ値の解析
            gyro_x = (data[22] << 8) + data[23]
            if gyro_x & 0x8000:  # 符号ビットが1の場合
                gyro_x -= 0x10000
            gyro_y = (data[24] << 8) + data[25]
            if gyro_y & 0x8000:  # 符号ビットが1の場合
                gyro_y -= 0x10000
            gyro_z = (data[26] << 8) + data[27]
            if gyro_z & 0x8000:  # 符号ビットが1の場合
                gyro_z -= 0x10000
            self.gyro['x'] = gyro_x / 1000
            self.gyro['y'] = gyro_y / 1000
            self.gyro['z'] = gyro_z / 1000
            
            # バッテリー電圧の解析
            battery_vol = (data[28] << 8) + data[29]
            self.battery = battery_vol / 1000
            
            if data_len == 34:
              # 高度の解析
              altitude = (data[30] << 8) + data[31]
              if altitude & 0x8000:  # 符号ビットが1の場合
                altitude -= 0x10000
              self.altitude = altitude / 1000

              # 温度の解析
              temperature = (data[32] << 8) + data[33]
              self.temperature = temperature / 1000
            return True
        return False