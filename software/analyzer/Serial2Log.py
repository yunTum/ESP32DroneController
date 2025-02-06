import serial
import datetime
import csv
import time
import keyboard

# シリアルポートの設定
SERIAL_PORT = 'COM18'  # 適切なポートに変更してください
BAUD_RATE = 115200

def save_log():
    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    filename = f'../Log/drone_log_{timestamp}.csv'

    try:
        # シリアルポートを開く
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
        print(f"Connected to {SERIAL_PORT}")
        print("Press 'l' to start/stop logging")
        print("Press 'q' to quit")
        
        csv_file = None
        csv_writer = None
        is_logging = False
        
        while True:
            # キーボード入力のチェック
            if keyboard.is_pressed('l'):
                is_logging = not is_logging
                if is_logging:
                    # 新しいCSVファイルを作成
                    csv_file = open(filename, 'w', newline='')
                    csv_writer = csv.writer(csv_file)
                    print(f"Started logging to {filename}")
                else:
                    if csv_file is not None:
                        csv_file.close()
                        csv_file = None
                    print("Logging stopped")
                # 'L'コマンドを送信
                ser.write(b'L')
                time.sleep(0.5)  # チャタリング防止
            
            if keyboard.is_pressed('q'):
                ser.write(b' ')
                print("\nQuitting...")
                break
                
            if ser.in_waiting:
                line = ser.readline().decode('utf-8').strip()
                
                # DATAで始まる行の処理
                if line.startswith('DATA,'):
                    # ヘッダー行の場合
                    if 'gyroADC_X' in line:
                        headers = line.replace('DATA,', '').split(',')
                        if csv_writer is not None:
                            csv_writer.writerow(headers)
                    
                    # データ行の場合
                    elif csv_writer is not None:
                        data = line.replace('DATA,', '').split(',')
                        csv_writer.writerow(data)
                        csv_file.flush()
                
                # ロギング中でない場合は通常のシリアル出力を表示
                elif not is_logging:
                    print(line)
                
    except serial.SerialException as e:
        print(f"Error: {e}")
    finally:
        if 'csv_file' in locals() and csv_file is not None:
            csv_file.close()
        if 'ser' in locals() and ser.is_open:
            ser.close()
        print("Program terminated")

if __name__ == "__main__":
    # keyboardライブラリのインストールが必要な場合
    try:
        import keyboard
    except ImportError:
        print("keyboardライブラリをインストールしてください:")
        print("pip install keyboard")
        exit(1)
    
    save_log()