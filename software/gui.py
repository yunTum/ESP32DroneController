import tkeasygui as te
import network
import threading
import data_define

class ControllerWindow():
  def __init__(self):
    self.client = network.TcpClient()
    # 送信のみのため、今のところ非同期処理を加えない
    # self.network_thread = threading.Thread(target=self.client.server_loop)
    self.create_window()
    self.resource_data = data_define.RemoteData()
    self.server_state = False

  def create_window(self):
    # ウィンドウの生成
    self.window = te.Window('Remote Control')

    # ボタンとスライダーの追加
    self.connect_button = te.Button(self.window, text='Connect', command=self.connect)
    self.disconnect_button = te.Button(self.window, text='Disconnect', command=self.disconnect, state='disabled')
    self.roll_slider = te.Scale(self.window, from_=256, to=2024, orient='horizontal', command=self.update_roll)
    self.pitch_slider = te.Scale(self.window, from_=256, to=2024, orient='vertical', command=self.update_pitch)
    self.rudder_slider = te.Scale(self.window, from_=256, to=2024, orient='horizontal', command=self.update_rudder)
    self.throttle_slider = te.Scale(self.window, from_=256, to=2024, orient='vertical', command=self.update_throttle)

      # レイアウトの配置
      self.connect_button.pack()
      self.disconnect_button.pack()
      self.roll_slider.pack()
      self.pitch_slider.pack()
      self.rudder_slider.pack()
      self.throttle_slider.pack()
    
  def run(self):
    # self.network_thread.start()
    # イベントループ
    while True:
        event, values = self.window.read()
        if event == sg.WINDOW_CLOSED:
            break
        if event == '-CONNECT-':
          if (not self.server_state):
            self.client.connect()
            self.server_state = True
            self.window['-CONNECT-'].update(disabled=True)
            self.window['-DISCONNECT-'].update(disabled=False)
            print('Server open')

        elif event == '-DISCONNECT-':
          if (self.server_state):
            self.client.close()
            self.server_state = False
            self.window['-CONNECT-'].update(disabled=False)
            self.window['-DISCONNECT-'].update(disabled=True)
            print('Server close')

        elif event == '-PITCH-':
          val = values['-PITCH-']
          self.resource_data.ch3 = val
          send_data = 'CH2:' + str(self.resource_data.ch3) + '\r'
          self.client.send(send_data.encode())
          print(f'PITCH: {val}')

        elif event == '-ROLL-':
          val = values['-ROLL-']
          self.resource_data.ch4 = val
          send_data = 'CH1:' + str(self.resource_data.ch4) + '\r'
          self.client.send(send_data.encode())
          print(f'ROLL: {val}')

        elif event == '-THROTTLE-':
          val = values['-THROTTLE-']
          self.resource_data.ch3 = val
          send_data = 'CH3:' + str(self.resource_data.ch3) + '\r'
          self.client.send(send_data.encode())
          print(f'Throttle: {val}')

        elif event == '-RUDDER-':
          val = values['-RUDDER-']
          self.resource_data.ch4 = val
          send_data = 'CH4:' + str(self.resource_data.ch4) + '\r'
          self.client.send(send_data.encode())
          print(f'Rudder: {val}')
        
        elif event == '-RADIO5-':
          if (self.resource_data.ch5):
            self.resource_data.ch5 = False
            self.window['-SWRADIO5-'].update("OFF")
            val = 256
          else:
            self.resource_data.ch5 = True
            self.window['-SWRADIO5-'].update("ON")
            val = 2000
          self.btn_send(5, val)
        
        elif event == '-RADIO6-':
          if (self.resource_data.ch6):
            self.resource_data.ch6 = False
            self.window['-SWRADIO6-'].update("OFF")
            val = 256
          else:
            self.resource_data.ch6 = True
            self.window['-SWRADIO6-'].update("ON")
            val = 2000
          self.btn_send(6, val)
          
        elif event == '-RADIO7-':
          if (self.resource_data.ch7):
            self.resource_data.ch7 = False
            self.window['-SWRADIO7-'].update("OFF")
            val = 256
          else:
            self.resource_data.ch7 = True
            self.window['-SWRADIO7-'].update("ON")
            val = 2000
          self.btn_send(7, val)
          
        elif event == '-RADIO8-':
          if (self.resource_data.ch8):
            self.resource_data.ch8 = False
            self.window['-SWRADIO8-'].update("OFF")
            val = 256
          else:
            self.resource_data.ch8 = True
            self.window['-SWRADIO8-'].update("ON")
            val = 2000
          self.btn_send(8, val)

    # ウィンドウを閉じる
    self.window.close()
  
  def btn_send(self, channel, val):
    send_data = 'CH' + str(channel) + ':' + str(val) + '\r'
    self.client.send(send_data.encode())
    print('Radio' + str(channel))

def main():
  gui = ControllerWindow()

if __name__ == '__main__':
  main()