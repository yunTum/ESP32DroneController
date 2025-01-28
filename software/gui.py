import TkEasyGUI as eg
import network
import threading
import data_define
import socket
class ControllerWindow():
  def __init__(self):
    self.client = None
    # 送信のみのため、今のところ非同期処理を加えない
    # self.network_thread = threading.Thread(target=self.client.server_loop)
    self.create_window()
    self.resource_data = data_define.RemoteData()
    self.drone_data = data_define.DroneData()
    self.server_state = False
    self.receive_thread = None

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
              eg.Slider(range=(1600,50), default_value=50, orientation='v', size=(10, 20), key='-THROTTLE-',  enable_events=True)],
          ]),
          eg.Frame('Channel', layout=[
              [eg.Button('ARM', key='-ARM-'), eg.Text('DISARMED', key='-ARM-', size=(10, 0))],
              [eg.Button('Radio6', key='-RADIO6-'), eg.Text('OFF', key='-SWRADIO6-')],
              [eg.Button('Radio7', key='-RADIO7-'), eg.Text('OFF', key='-SWRADIO7-')],
              [eg.Button('Default', key='-Default-')],
          ]),
          eg.Frame('Drone Info', layout=[
                [eg.Text('Battery: ---V', key='-BATTERY-')],
                [eg.Text('Servo1: ---', key='-SERVO1-')],
                [eg.Text('Servo2: ---', key='-SERVO2-')],
                [eg.Text('Servo3: ---', key='-SERVO3-')],
                [eg.Text('Servo4: ---', key='-SERVO4-')],
                [eg.Text('IMU Roll: ---', key='-IMUROLL-')],
                [eg.Text('IMU Pitch: ---', key='-IMUPITCH-')],
                [eg.Text('IMU Yaw: ---', key='-IMUYAW-')],
                [eg.Text('Gyro X: ---', key='-GYROX-')],
                [eg.Text('Gyro Y: ---', key='-GYROY-')],
                [eg.Text('Gyro Z: ---', key='-GYROZ-')],
            ]),
        ]
    ]
    # ウィンドウの生成
    self.window = eg.Window('Remote Control', self.layout, size=(1300, 400))
    
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
            # 受信スレッドの開始
            self.receive_thread = threading.Thread(target=self.receive_udp)
            self.receive_thread.daemon = True
            self.receive_thread.start()
            print('Server Opened')

        elif event == '-DISCONNECT-':
          if (self.server_state):
            if self.client:
              self.client.close()
              self.client = None
            if self.receive_thread:
              self.receive_thread.join(timeout=2.0) 
            self.server_state = False
            self.window['-CONNECT-'].update(disabled=False)
            self.window['-DISCONNECT-'].update(disabled=True)
            print('Server Closed')

        elif event == '-PITCH-':
          val = values['-PITCH-']
          self.resource_data.rc_value[2] = int(val)
          # print(f'PITCH: {val}')

        elif event == '-ROLL-':
          val = values['-ROLL-']
          self.resource_data.rc_value[1] = int(val)
          # print(f'ROLL: {val}')

        elif event == '-THROTTLE-':
          val = values['-THROTTLE-']
          self.resource_data.rc_value[0] = int(val)
          # print(f'Throttle: {val}')

        elif event == '-RUDDER-':
          val = values['-RUDDER-']
          self.resource_data.rc_value[3] = int(val)
          # print(f'Rudder: {val}')
        
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
          if (self.resource_data.rc_value[6] > 1615):
            self.resource_data.rc_value[6] = 1515
            self.window['-SWRADIO7-'].update("OFF")
          else:
            self.resource_data.rc_value[6] = 2015
            self.window['-SWRADIO7-'].update("ON")
          
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

  def receive_udp(self):
    print("Receive thread start") 
    while self.server_state:
      try:
          data, addr = self.client.udp_server.recvfrom(self.client.read_size)
          # print(f"Recv Size: {len(data)} bytes")
          # print(f"Receive: {addr} from")
          # print(f"Battery: {self.drone_data.battery}")
          # print(f"IMU Roll: {self.drone_data.imu['roll']}")
          if len(data) != self.client.read_size:
            print(f"Invalid data size: {len(data)} bytes, expected 30 bytes")
            continue
          if not self.client.remote_address:
            self.client.remote_address = addr
            print(f"Send Address is set: {addr}")
          # if self.drone_data.parse_data(data):
          #     # データの更新が成功したら必要な処理を行う
          #     # 例：表示の更新など
          #     print(f"Battery: {self.drone_data.battery}")
          #     print(f"IMU Roll: {self.drone_data.imu['roll']}")
          if self.drone_data.parse_data(data):
              # GUIの更新
              self.window['-BATTERY-'].update(f"Battery: {self.drone_data.battery:.2f}V")
              self.window['-SERVO1-'].update(f"Servo1: {self.drone_data.servo[0]:.1f}")
              self.window['-SERVO2-'].update(f"Servo2: {self.drone_data.servo[1]:.1f}")
              self.window['-SERVO3-'].update(f"Servo3: {self.drone_data.servo[2]:.1f}")
              self.window['-SERVO4-'].update(f"Servo4: {self.drone_data.servo[3]:.1f}")
              self.window['-IMUROLL-'].update(f"Roll: {self.drone_data.imu['roll']:.1f}")
              self.window['-IMUPITCH-'].update(f"Pitch: {self.drone_data.imu['pitch']:.1f}")
              self.window['-IMUYAW-'].update(f"Yaw: {self.drone_data.imu['yaw']:.1f}")
              self.window['-GYROX-'].update(f"Gyro X: {self.drone_data.gyro['x']:.1f}")
              self.window['-GYROY-'].update(f"Gyro Y: {self.drone_data.gyro['y']:.1f}")
              self.window['-GYROZ-'].update(f"Gyro Z: {self.drone_data.gyro['z']:.1f}")
      except socket.timeout:
          # タイムアウトは正常なので、エラーメッセージを表示しない
          continue
      except Exception as e:
          if not self.server_state:
              break
          print('Receive failed:', str(e))  # エラーの詳細を出力
          import traceback
          traceback.print_exc()  # スタックトレースを出力
def main():
  gui = ControllerWindow()

if __name__ == '__main__':
  main()