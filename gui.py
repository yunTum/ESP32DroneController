import TkEasyGUI as eg
import network
import threading
import data_define

class ControllerWindow():
  def __init__(self):
    self.client = None
    # 送信のみのため、今のところ非同期処理を加えない
    # self.network_thread = threading.Thread(target=self.client.server_loop)
    self.create_window()
    self.resource_data = data_define.RemoteData()
    self.server_state = False

  def create_window(self):
    # レイアウトの定義
    self.layout = [
        [
          eg.Frame('Network', layout=[
                [eg.Button('Connect', key='-CONNECT-', disabled=False)],
                [eg.Button('Disconnect', key='-DISCONNECT-', disabled=True)]
          ]),
          eg.Frame('Roll & Pitch', layout=[
              [eg.Slider(range=(1015, 2015), default_value=1515, orientation='h', size=(20, 10), key='-ROLL-',  enable_events=True),
              eg.Slider(range=(2015,1015), default_value=1515, orientation='v', size=(10, 20), key='-PITCH-',  enable_events=True)],
          ]),
          eg.Frame('Rudder & Throttle', layout=[
              [eg.Slider(range=(1015, 2015), default_value=1515, orientation='h', size=(20, 10), key='-RUDDER-',  enable_events=True),
              eg.Slider(range=(1100,50), default_value=50, orientation='v', size=(10, 20), key='-THROTTLE-',  enable_events=True)],
          ]),
          eg.Frame('Channel', layout=[
              [eg.Button('ARM', key='-ARM-'), eg.Text('DISARMED', key='-ARM-', size=(10, 0))],
              [eg.Button('Radio6', key='-RADIO6-'), eg.Text('OFF', key='-SWRADIO6-')],
              [eg.Button('Radio7', key='-RADIO7-'), eg.Text('OFF', key='-SWRADIO7-')],
              [eg.Button('Default', key='-Default-')],
          ]),
        ]
    ]
    # ウィンドウの生成
    self.window = eg.Window('Remote Control', self.layout, size=(1100, 200))
    
  def run(self):
    # self.network_thread.start()
    # イベントループ
    while True:
        event, values = self.window.read()
        if event == eg.WINDOW_CLOSED:
            break
        if event == '-CONNECT-':
          if (not self.server_state):
            self.client = network.TcpClient()
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
          self.resource_data.rc_value[2] = int(val)
          print(f'PITCH: {val}')

        elif event == '-ROLL-':
          val = values['-ROLL-']
          self.resource_data.rc_value[1] = int(val)
          print(f'ROLL: {val}')

        elif event == '-THROTTLE-':
          val = values['-THROTTLE-']
          self.resource_data.rc_value[0] = int(val)
          print(f'Throttle: {val}')

        elif event == '-RUDDER-':
          val = values['-RUDDER-']
          self.resource_data.rc_value[3] = int(val)
          print(f'Rudder: {val}')
        
        elif event == '-ARM-':
          if (self.resource_data.rc_value[4] > 1615):
            self.resource_data.rc_value[4] = 1515
            self.window['-ARM-'].update("DISARMED")
          else:
            self.resource_data.rc_value[4] = 2015
            self.window['-ARM-'].update("ARMED")
        
        elif event == '-RADIO6-':
          if (self.resource_data.rc_value[5] > 1615):
            self.resource_data.rc_value[5] = 1515
            self.window['-SWRADIO6-'].update("OFF")
          else:
            self.resource_data.rc_value[5] = 2015
            self.window['-SWRADIO6-'].update("ON")
          
        elif event == '-RADIO7-':
          pass
          
        elif event == '-Default-':
          self.resource_data.defalut_set()
          self.window['-PITCH-'].update(value=1515)
          self.window['-ROLL-'].update(value=1515)
          self.window['-THROTTLE-'].update(value=50)
          self.window['-RUDDER-'].update(value=1515)
          self.resource_data.rc_value[4] = 1515
          self.window['-ARM-'].update("DISARMED")
          self.resource_data.rc_value[5] = 1515
          self.window['-SWRADIO6-'].update("OFF")
        
        if (self.server_state):
          self.send_udp()

    # ウィンドウを閉じる
    self.window.close()
  
  def send_udp(self):
    send_data = self.resource_data.create_byte()
    self.client.send(send_data)

def main():
  gui = ControllerWindow()

if __name__ == '__main__':
  main()