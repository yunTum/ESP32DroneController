# 構造体として使用
class RemoteData():
  def __init__(self):
    self.chnum = 6
    self.rc_value = [1515] * self.chnum
    self.seqno = 0
    
  def store_RC(self, in_value, out, offset):
    out[offset] = (in_value >> 8) & 0xFF  # 上位バイト
    out[offset + 1] = in_value & 0xFF     # 下位バイト
  
  def create_byte(self):
    buff = bytearray(14)
    buff[0] = 0x55
    buff[1] = self.seqno
    self.store_RC(self.rc_value[1], buff,  2)
    self.store_RC(self.rc_value[2], buff,  4)
    self.store_RC(self.rc_value[0], buff,  6)
    self.store_RC(self.rc_value[3], buff,  8)
    self.store_RC(self.rc_value[4], buff, 10)
    self.store_RC(self.rc_value[5], buff, 12)
    return buff
  
  def defalut_set(self):
    self.rc_value = [1515] * self.chnum