import TkEasyGUI as eg
import network
import threading
import data_define
import socket
import tkinter as tk
import time

from viewer import DroneViewer
class ControllerWindow():
  def __init__(self):
    self.client = None
    self.create_window()
    self.resource_data = data_define.RemoteData()
    self.drone_data = data_define.DroneData()
    self.server_state = False
    self.receive_thread = None
    # 3Dビューアの有効無効
    self.is_3d_viewer = True

    if self.is_3d_viewer:
      # DroneViewerの初期化
      # PySimpleGUIのウィンドウからTkinterのルートウィンドウを取得
      canvas = self.window['-CANVAS-'].TKCanvas
      # フレームを作成してその中にDroneViewerを配置
      frame = tk.Frame(canvas.master)
      frame.pack(fill=tk.BOTH, expand=True)
      
      self.viewer = DroneViewer(frame)
      self.viewer.pack(fill=tk.BOTH, expand=True)


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
              [eg.Button('Calibrate', key='-CALIBRATE-'), eg.Text('OFF', key='-CALIBRATE-')],
              [eg.Button('Default', key='-Default-')],
          ]),
          eg.Frame('Drone Info', layout=[
                [eg.Text('Battery: ---V', key='-BATTERY-')],
                [eg.Text('Servo1: ---', key='-SERVO1-')],
                [eg.Text('Servo2: ---', key='-SERVO2-')],
                [eg.Text('Servo3: ---', key='-SERVO3-')],
                [eg.Text('Servo4: ---', key='-SERVO4-')],
                [eg.Text('IMU Roll: ---', key='-IMUROLL-', size=(10, 0)), eg.Text('PID Roll: ---', key='-PIDROLL-', size=(10, 0), pad=(30, 0))],
                [eg.Text('IMU Pitch: ---', key='-IMUPITCH-', size=(10, 0)), eg.Text('PID Pitch: ---', key='-PIDPITCH-', size=(10, 0), pad=(30, 0))],
                [eg.Text('IMU Yaw: ---', key='-IMUYAW-', size=(10, 0)), eg.Text('PID Yaw: ---', key='-PIDYAW-', size=(10, 0), pad=(30, 0))],
                [eg.Text('I-P Roll: ---', key='-IPROLL-', size=(10, 0))],
                [eg.Text('I-P Pitch: ---', key='-IPPITCH-', size=(10, 0))],
                [eg.Text('I-P Yaw: ---', key='-IPYAW-', size=(10, 0))],
                [eg.Text('Gyro X: ---', key='-GYROX-', size=(10, 0)), eg.Text('Acc X: ---', key='-ACCX-', size=(10, 0), pad=(30, 0))],
                [eg.Text('Gyro Y: ---', key='-GYROY-', size=(10, 0)), eg.Text('Acc Y: ---', key='-ACCY-', size=(10, 0), pad=(30, 0))],
                [eg.Text('Gyro Z: ---', key='-GYROZ-', size=(10, 0)), eg.Text('Acc Z: ---', key='-ACCZ-', size=(10, 0), pad=(30, 0))],
                [eg.Text('Altitude: ---m', key='-ALTITUDE-')],
                [eg.Text('Temperature: ---℃', key='-TEMPERATURE-')],
            ]),
            # OpenGL用のキャンバスを追加
            eg.Frame('3D View', layout=[
                [eg.Canvas(size=(60, 400), key='-CANVAS-')]
            ]),
        ]
    ]
    # ウィンドウの生成
    self.window = eg.Window('Remote Control', self.layout, finalize=True, resizable=True, size=(1550, 450))
    
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
          
        elif event == '-CALIBRATE-':
          # if (self.resource_data.rc_value[7] > 1615):
            self.resource_data.rc_value[7] = 2015
            self.window['-CALIBRATE-'].update(disabled=True)
          # else:
            # キャリブレーション完了を待機
            # while self.resource_data.rc_value[7] > 1615:
            #     time.sleep(0.1)  # 完了待ち
            # self.resource_data.rc_value[7] = 2015
            self.window['-CALIBRATE-'].update(disabled=False)
            # self.resource_data.rc_value[7] = 1515
          
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
    self.resource_data.rc_value[7] = 1515

  def receive_udp(self):
    print("Receive thread start") 
    while self.server_state:
      try:
          data, addr = self.client.udp_server.recvfrom(self.client.read_size)
          if len(data) != self.client.read_size:
            print(f"Invalid data size: {len(data)} bytes, expected 30 bytes")
            continue
          if not self.client.remote_address:
            self.client.remote_address = addr
            print(f"Send Address is set: {addr}")
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
              self.window['-PIDROLL-'].update(f"PID Roll: {self.drone_data.pid['roll']:.1f}")
              self.window['-PIDPITCH-'].update(f"PID Pitch: {self.drone_data.pid['pitch']:.1f}")
              self.window['-PIDYAW-'].update(f"PID Yaw: {self.drone_data.pid['yaw']:.1f}")
              self.window['-GYROX-'].update(f"Gyro X: {self.drone_data.gyro['x']:.1f}")
              self.window['-GYROY-'].update(f"Gyro Y: {self.drone_data.gyro['y']:.1f}")
              self.window['-GYROZ-'].update(f"Gyro Z: {self.drone_data.gyro['z']:.1f}")
              self.window['-ALTITUDE-'].update(f"Altitude: {self.drone_data.altitude:.1f}m")
              self.window['-TEMPERATURE-'].update(f"Temperature: {self.drone_data.temperature:.1f}℃")
              self.window['-ACCX-'].update(f"Acc X: {self.drone_data.acc['x']:.1f}")
              self.window['-ACCY-'].update(f"Acc Y: {self.drone_data.acc['y']:.1f}")
              self.window['-ACCZ-'].update(f"Acc Z: {self.drone_data.acc['z']:.1f}")
              self.window['-IPROLL-'].update(f"I-P Roll: {self.drone_data.ip['roll']:.1f}")
              self.window['-IPPITCH-'].update(f"I-P Pitch: {self.drone_data.ip['pitch']:.1f}")
              self.window['-IPYAW-'].update(f"I-P Yaw: {self.drone_data.ip['yaw']:.1f}")
              if self.is_3d_viewer:
                # 3Dビューの更新
                self.viewer.after(0, self.update_viewer)

          else:
            print(f"Parse failed: {data} {len(data)}")

      except socket.timeout:
          # タイムアウトは正常なので、エラーメッセージを表示しない
          continue
      except Exception as e:
          if not self.server_state:
              break
          print('Receive failed:', str(e))  # エラーの詳細を出力
          import traceback
          traceback.print_exc()  # スタックトレースを出力

  def update_viewer(self):
    """3Dビューアのデータを更新（メインスレッドで実行）"""
    try:
        self.viewer.update_attitude(
            self.drone_data.imu['roll'],
            self.drone_data.imu['pitch'],
            self.drone_data.imu['yaw']
        )
    except Exception as e:
        print(f"ビューア更新エラー: {str(e)}")

def main():
  gui = ControllerWindow()

if __name__ == '__main__':
  main()