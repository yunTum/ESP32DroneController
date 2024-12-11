import socket

class TcpClient():
  def __init__(self):
    self.server_ip = '127.0.0.1'
    self.server_port = 4211
    # self.listen_num = 5
    self.buffer_size = 1024
    self.read_size = 30
    self.client_ip = '192.168.40.234' # シリアル出力で表示されるIPを用いる
    self.client_port = 4210
    self.udp_server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.udp_server.bind(('0.0.0.0', self.server_port))  # 受信用にバインド
    self.udp_server.settimeout(1.0)
    self.client_address = (self.client_ip, self.client_port)
    self.udp_server.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.read_size)
    self.remote_address = None
  
  def send(self, data):
    try:
      if self.remote_address:
        self.udp_server.sendto(data, self.remote_address)
      else:
        # 従来の送信先にフォールバック
        self.udp_server.sendto(data, self.client_address)
    except:
      print('send failed')
  
  def close(self):
    self.udp_server.close()