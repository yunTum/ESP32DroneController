# 機体情報を管理
# パケットを作成
class RemoteData():
  def __init__(self):
    self.chnum = 6
    self.rc_value = [1515] * self.chnum
    self.rc_value[0] = 50 #Range 50 - 900
    self.seqno = 0
    
  def store_RC(self, in_value, out, offset):
    out[offset] = (in_value >> 8) & 0xFF  # 上位バイト
    out[offset + 1] = in_value & 0xFF     # 下位バイト
  
  def create_byte(self):
    buff = bytearray(14)
    buff[0] = 0x55                            # チェック値
    buff[1] = self.seqno                      # シーケンス番号
    self.store_RC(self.rc_value[1], buff,  2) # ROLL
    self.store_RC(self.rc_value[2], buff,  4) # PITCH
    self.store_RC(self.rc_value[0], buff,  6) # THROTTLE
    self.store_RC(self.rc_value[3], buff,  8) # RUDDER
    self.store_RC(self.rc_value[4], buff, 10) # ARM
    self.store_RC(self.rc_value[5], buff, 12) # RADIO6
    return buff
  
  def defalut_set(self):
    self.rc_value = [1515] * self.chnum
    self.rc_value[0] = 50