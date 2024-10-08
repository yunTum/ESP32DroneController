# Not using

import asyncio, socket

class tcp_client():
  def __init__(self):
    self.server_ip = "0.0.0.0"
    self.server_port = 6342
    self.listen_num = 5
    self.buffer_size = 1024
    self.client = None
    self.client_address = None
    self.tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

  async def server_loop(self):
    self.tcp_server.bind((self.server_ip, self.server_port))
    self.tcp_server.listen(self.listen_num)
    self.tcp_server.setblocking(False)
    loop = asyncio.get_event_loop()
    while True:
      self.client, self.client_address = await loop.sock_accept(self.tcp_server)
      self.client.setblocking(False)
      asyncio.create_task(self.handle_client(self.client, loop))
        
  async def handle_client(self, client, loop):
    while data := await loop.sock_recv(client, 1024):
        print( f"data = {data}")
        await loop.sock_sendall(client, b'ACK!!\r')
    client.close()
    
  def run(self):
    asyncio.run(self.server_loop())
    
def main():
  client = tcp_client()
  client.server_loop()

if __name__ == '__main__':
  main()