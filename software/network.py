import socket

class TcpClient():
  def __init__(self):
    self.server_ip = '127.0.0.1'
    self.server_port = 4211
    # self.listen_num = 5
    self.buffer_size = 1024
    self.client_ip = '192.168.40.231'
    self.client_port = 4210
    self.udp_server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.client_address = (self.client_ip, self.client_port)

  def connect(self):
    # self.client, self.client_address = self.udp_server.accept()
    pass
  def server_loop(self):
    while True:
      try:
        data, cli_addr  = self.udp_server.recvfrom(self.buffer_size)
        print("[*] Received Data : {}".format(data))
      except:
        self.udp_server.close()
  
  def send(self, data):
    try:
      self.udp_server.sendto(data, self.client_address)
    except:
      print('send failed')
  
  def close(self):
    self.udp_server.close()