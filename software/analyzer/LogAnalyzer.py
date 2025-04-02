import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
import matplotlib.dates as mdates
from datetime import datetime
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                           QHBoxLayout, QPushButton, QLabel, QFileDialog, 
                           QComboBox, QMessageBox, QScrollArea, QTabWidget)
from PyQt5.QtCore import Qt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import sys

def analyze_log(file_path):
    # CSVファイルを読み込む
    df = pd.read_csv(file_path)
    
    # 基本的な統計情報を表示
    print("\n基本統計情報:")
    print(df.describe())
    
    # プロットの設定
    plt.rcParams['figure.figsize'] = [12, 6]
    plt.rcParams['grid.alpha'] = 0.3
    plt.rcParams['grid.color'] = '#cccccc'
    
    # IMUの角度データをプロット
    plt.figure()
    plt.plot(df['roll_IMU'], label='Roll')
    plt.plot(df['pitch_IMU'], label='Pitch')
    plt.plot(df['yaw_IMU'], label='Yaw')
    plt.title('IMU Angles')
    plt.xlabel('Sample')
    plt.ylabel('Angle (deg)')
    plt.legend()
    plt.grid(True)
    
    # PIDの出力をプロット
    plt.figure()
    plt.plot(df['roll_PID'], label='Roll PID')
    plt.plot(df['pitch_PID'], label='Pitch PID')
    plt.plot(df['yaw_PID'], label='Yaw PID')
    plt.title('PID Outputs')
    plt.xlabel('Sample')
    plt.ylabel('Output')
    plt.legend()
    plt.grid(True)
    
    # ジャイロデータをプロット
    plt.figure()
    plt.plot(df['GyroX'], label='X')
    plt.plot(df['GyroY'], label='Y')
    plt.plot(df['GyroZ'], label='Z')
    plt.title('Gyro Data')
    plt.xlabel('Sample')
    plt.ylabel('Angular Velocity (deg/s)')
    plt.legend()
    plt.grid(True)
    
    # 加速度データをプロット
    plt.figure()
    plt.plot(df['AccX'], label='X')
    plt.plot(df['AccY'], label='Y')
    plt.plot(df['AccZ'], label='Z')
    plt.title('Accelerometer Data')
    plt.xlabel('Sample')
    plt.ylabel('Acceleration (m/s²)')
    plt.legend()
    plt.grid(True)
    
    # 全てのグラフを表示
    plt.show()
    
    # 追加の統計情報
    print("\nIMU角度の平均値:")
    print(f"Roll: {df['roll_IMU'].mean():.2f}°")
    print(f"Pitch: {df['pitch_IMU'].mean():.2f}°")
    print(f"Yaw: {df['yaw_IMU'].mean():.2f}°")
    
    print("\nPID出力の標準偏差:")
    print(f"Roll: {df['roll_PID'].std():.2f}")
    print(f"Pitch: {df['pitch_PID'].std():.2f}")
    print(f"Yaw: {df['yaw_PID'].std():.2f}")

def analyze_phase(df):
    # IMUとPIDの相関を計算
    roll_corr = np.correlate(df['roll_IMU'], df['roll_PID'], mode='full')
    pitch_corr = np.correlate(df['pitch_IMU'], df['pitch_PID'], mode='full')
    yaw_corr = np.correlate(df['yaw_IMU'], df['yaw_PID'], mode='full')
    
    # IMUとPIDの比較プロット（正規化して表示）
    plt.figure(figsize=(15, 15))  # サイズを大きくして4つのサブプロットを配置
    
    # Roll
    plt.subplot(3, 1, 1)
    roll_imu_norm = df['roll_IMU'] / df['roll_IMU'].abs().max()
    roll_pid_norm = df['roll_PID'] / df['roll_PID'].abs().max()
    gyro_x_norm = df['GyroX'] / df['GyroX'].abs().max()
    plt.plot(gyro_x_norm, label='Gyro X (normalized)')
    # plt.plot(roll_imu_norm, label='Roll IMU (normalized)')
    plt.plot(roll_pid_norm, label='Roll PID (normalized)')

    # plt.plot(roll_imu_norm + roll_pid_norm, label='Combined', alpha=0.5)
    plt.plot(gyro_x_norm + roll_pid_norm, label='Combined', alpha=0.5)
    # plt.title('Roll: IMU vs PID (Normalized)')
    plt.title('Roll: Gyro X vs PID (Normalized)')

    plt.legend()
    plt.grid(True)
    
    # Pitch
    plt.subplot(3, 1, 2)
    pitch_imu_norm = df['pitch_IMU'] / df['pitch_IMU'].abs().max()
    pitch_pid_norm = df['pitch_PID'] / df['pitch_PID'].abs().max()
    gyro_y_norm = df['GyroY'] / df['GyroY'].abs().max()

    plt.plot(gyro_y_norm, label='Gyro Y (normalized)')
    # plt.plot(pitch_imu_norm, label='Pitch IMU (normalized)')
    plt.plot(pitch_pid_norm, label='Pitch PID (normalized)')
    plt.plot(gyro_y_norm + pitch_pid_norm, label='Combined', alpha=0.5)

    # plt.plot(pitch_imu_norm + pitch_pid_norm, label='Combined', alpha=0.5)
    # plt.title('Pitch: IMU vs PID (Normalized)')
    plt.title('Pitch: Gyro Y vs PID (Normalized)')
    plt.legend()
    plt.grid(True)
    

    # Yaw
    plt.subplot(3, 1, 3)
    yaw_imu_norm = df['yaw_IMU'] / df['yaw_IMU'].abs().max()
    yaw_pid_norm = df['yaw_PID'] / df['yaw_PID'].abs().max()
    gyro_z_norm = df['GyroZ'] / df['GyroZ'].abs().max()
    plt.plot(gyro_z_norm, label='Gyro Z (normalized)')

    # plt.plot(yaw_imu_norm, label='Yaw IMU (normalized)')
    plt.plot(yaw_pid_norm, label='Yaw PID (normalized)')
    plt.plot(gyro_z_norm + yaw_pid_norm, label='Combined', alpha=0.5)
    # plt.plot(yaw_imu_norm + yaw_pid_norm, label='Combined', alpha=0.5)

    # plt.title('Yaw: IMU vs PID (Normalized)')
    plt.title('Yaw: Gyro Z vs PID (Normalized)')
    plt.legend()
    plt.grid(True)
    

    # # 合成波形の比較
    # plt.subplot(4, 1, 4)
    # plt.plot(roll_imu_norm + roll_pid_norm, label='Roll Combined', alpha=0.7)
    # plt.plot(pitch_imu_norm + pitch_pid_norm, label='Pitch Combined', alpha=0.7)
    # plt.plot(yaw_imu_norm + yaw_pid_norm, label='Yaw Combined', alpha=0.7)
    # plt.title('Combined Waveforms Comparison')
    # plt.legend()
    # plt.grid(True)
    
    plt.tight_layout()
    plt.show()
    
    # 相関係数と合成波形の統計を計算
    print("\nIMUとPIDの相関係数（-1に近いほど良い）:")
    print(f"Roll: {df['GyroX'].corr(df['roll_PID']):.3f}")
    print(f"Pitch: {df['GyroY'].corr(df['pitch_PID']):.3f}")
    print(f"Yaw: {df['GyroZ'].corr(df['yaw_PID']):.3f}")

    print("\n合成波形の統計:")
    print("Roll合成波形の標準偏差:", (gyro_x_norm + roll_pid_norm).std().round(3))
    print("Pitch合成波形の標準偏差:", (gyro_y_norm + pitch_pid_norm).std().round(3))
    print("Yaw合成波形の標準偏差:", (gyro_z_norm + yaw_pid_norm).std().round(3))
    
    # 位相差の評価
    def calculate_phase_difference(imu, pid):
        correlation = np.correlate(imu, pid, mode='full')
        max_corr_index = np.argmax(np.abs(correlation))
        phase_diff = max_corr_index - len(imu)
        return phase_diff
    
    print("\n位相差（サンプル数）:")
    print(f"Roll: {calculate_phase_difference(gyro_x_norm, roll_pid_norm)}")
    print(f"Pitch: {calculate_phase_difference(gyro_y_norm, pitch_pid_norm)}")
    print(f"Yaw: {calculate_phase_difference(gyro_z_norm, yaw_pid_norm)}")


def analyze_pid_coefficients(df, CURRENT_PID):
    """IMUデータからPID係数を推定し、現在の設定と比較する関数"""
    dt = 0.02  # 20ms (LoopInterval)
    axes = ['roll', 'pitch', 'yaw']
    
    for axis in axes:
        print(f"\n=== {axis.upper()}軸の分析 ===")
        
        # データ取得
        imu_data = df[f'{axis}_IMU']
        pid_data = df[f'{axis}_PID']
        
        # 基本的な特性分析
        amplitude = (imu_data.max() - imu_data.min()) / 2
        zero_crossings = np.where(np.diff(np.signbit(imu_data - imu_data.mean())))[0]
        if len(zero_crossings) >= 2:
            avg_period = 2 * dt * len(imu_data) / len(zero_crossings)
            frequency = 1 / avg_period
        else:
            frequency = 0
            avg_period = 0

        # Rate制御のPIDゲイン推定
        Ku_rate = 4 * amplitude
        Tu_rate = avg_period if frequency > 0 else 0
        
        if Tu_rate > 0:
            Kp_rate = 0.6 * Ku_rate
            Ki_rate = 2 * Kp_rate / Tu_rate
            Kd_rate = Kp_rate * Tu_rate / 8
        else:
            Kp_rate = Ku_rate * 0.5
            Ki_rate = 0
            Kd_rate = 0

        # Angle制御のPIDゲイン推定
        # Angle制御は通常Rate制御よりも緩やかに設定
        Kp_angle = Kp_rate * 0.5  # 姿勢制御はレート制御の半分程度
        Ki_angle = Ki_rate * 0.1  # 積分項は小さく
        Kd_angle = Kd_rate * 0.2  # 微分項も小さく

        # 現在の設定値を取得
        current_rate = CURRENT_PID['rate']['yaw' if axis == 'yaw' else 'roll_pitch']
        current_angle = CURRENT_PID['angle']['yaw' if axis == 'yaw' else 'roll_pitch']
        
        print(f"\n振動特性:")
        print(f"- 振幅: {amplitude:.2f}°")
        print(f"- 周波数: {frequency:.2f} Hz")
        print(f"- 周期: {avg_period*1000:.1f} ms")
        
        print(f"\n現在の設定値:")
        print("Rate制御:")
        print(f"Kp = {current_rate['Kp']:.3f}")
        print(f"Ki = {current_rate['Ki']:.3f}")
        print(f"Kd = {current_rate['Kd']:.3f}")
        print("Angle制御:")
        print(f"Kp = {current_angle['Kp']:.3f}")
        print(f"Ki = {current_angle['Ki']:.3f}")
        print(f"Kd = {current_angle['Kd']:.3f}")
        
        print(f"\n推奨Rate PIDゲイン:")
        print(f"Kp = {Kp_rate:.3f} (現在比: {Kp_rate/current_rate['Kp']:.2f}倍)")
        print(f"Ki = {Ki_rate:.3f} (現在比: {Ki_rate/current_rate['Ki']:.2f}倍)")
        print(f"Kd = {Kd_rate:.3f} (現在比: {Kd_rate/current_rate['Kd']:.2f}倍)")

        print(f"\n推奨Angle PIDゲイン:")
        print(f"Kp = {Kp_angle:.3f} (現在比: {Kp_angle/current_angle['Kp']:.2f}倍)")
        print(f"Ki = {Ki_angle:.3f} (現在比: {Ki_angle/current_angle['Ki']:.2f}倍)")
        print(f"Kd = {Kd_angle:.3f} (現在比: {Kd_angle/current_angle['Kd']:.2f}倍)")
        
        # 応答性の評価
        response_time = np.where(np.abs(imu_data) < 0.1 * amplitude)[0]
        settling_time = len(response_time) * dt if len(response_time) > 0 else float('inf')
        overshoot = (amplitude - np.abs(imu_data.mean())) / np.abs(imu_data.mean()) * 100 if imu_data.mean() != 0 else 0
        
        print(f"\n性能評価:")
        print(f"安定性スコア: {1.0/(1.0 + amplitude * frequency):.2f} (1に近いほど安定)")
        print(f"整定時間: {settling_time*1000:.1f} ms")
        print(f"オーバーシュート: {overshoot:.1f}%")
        
        print("\n改善提案:")
        if amplitude > 10:
            print(f"- 振幅が大きすぎます（{amplitude:.1f}°）")
            print("  → Rate Kpを下げるか、Angle Kpを下げることを検討")
            print("  → まずはAngle Kpを現在の80%程度に下げることを推奨")
        if frequency > 5:
            print(f"- 振動が速すぎます（{frequency:.1f} Hz）")
            print("  → Rate Kdを上げることを検討")
            print("  → Angle Kdも若干上げることを検討")
        if settling_time > 0.5:
            print(f"- 整定時間が長すぎます（{settling_time*1000:.1f} ms）")
            print("  → Angle Kiを若干上げることを検討")
        if overshoot > 20:
            print(f"- オーバーシュートが大きすぎます（{overshoot:.1f}%）")
            print("  → Angle Kpを下げ、Kdを上げることを検討")
        if amplitude < 2 and frequency < 2 and overshoot < 10:
            print("- 現在の設定は安定しています")
            print("  → 必要に応じて慎重にゲインを上げることで応答性を改善可能")

# CSVファイルを読み込む
def load_log_data(file_path):
    df = pd.read_csv(file_path, parse_dates=['timestamp'])
    return df

def plot_flight_data(df, fig=None):
    """飛行データをプロットする関数
    Args:
        df: データフレーム
        fig: 既存のFigureオブジェクト（GUI表示用）
    """
    # 日本語フォントの設定
    plt.rcParams['font.family'] = 'MS Gothic'
    
    # 新しいFigureを作成するか、既存のものを使用
    if fig is None:
        fig, ((ax1, ax2), (ax3, ax4), (ax5, ax6)) = plt.subplots(3, 2, figsize=(15, 18))
    else:
        # 既存のFigureをクリア
        fig.clear()
        # サブプロットを作成
        ((ax1, ax2), (ax3, ax4), (ax5, ax6)) = fig.subplots(3, 2)
    
    # 時間軸のフォーマット設定
    time_format = mdates.DateFormatter('%H:%M:%S')
    
    # 1. IMU角度のプロット
    ax1.plot(df['timestamp'], df['imu_roll'], label='Roll')
    ax1.plot(df['timestamp'], df['imu_pitch'], label='Pitch')
    ax1.plot(df['timestamp'], df['imu_yaw'], label='Yaw')
    ax1.set_title('IMU Angle')
    ax1.set_ylabel('Angle (deg)')
    ax1.legend()
    ax1.grid(True)
    ax1.xaxis.set_major_formatter(time_format)
    
    # 2. PID出力のプロット
    ax2.plot(df['timestamp'], df['pid_roll'], label='Roll')
    ax2.plot(df['timestamp'], df['pid_pitch'], label='Pitch')
    ax2.plot(df['timestamp'], df['pid_yaw'], label='Yaw')
    ax2.set_title('PID Output')
    ax2.set_ylabel('Output value')
    ax2.legend()
    ax2.grid(True)
    ax2.xaxis.set_major_formatter(time_format)
    
    # 3. ジャイロデータのプロット
    ax3.plot(df['timestamp'], df['gyro_x'], label='X')
    ax3.plot(df['timestamp'], df['gyro_y'], label='Y')
    ax3.plot(df['timestamp'], df['gyro_z'], label='Z')
    ax3.set_title('Gyroscope')
    ax3.set_ylabel('Angular velocity (deg/s)')
    ax3.legend()
    ax3.grid(True)
    ax3.xaxis.set_major_formatter(time_format)
    
    # 4. 加速度データのプロット
    ax4.plot(df['timestamp'], df['acc_x'], label='X')
    ax4.plot(df['timestamp'], df['acc_y'], label='Y')
    ax4.plot(df['timestamp'], df['acc_z'], label='Z')
    ax4.set_title('Accelerometer')
    ax4.set_ylabel('Acceleration (m/s²)')
    ax4.legend()
    ax4.grid(True)
    ax4.xaxis.set_major_formatter(time_format)
    
    # 5. サーボ値のプロット
    ax5.plot(df['timestamp'], df['servo1'], label='Servo 1(RightBack)')
    ax5.plot(df['timestamp'], df['servo2'], label='Servo 2(RightTop)')
    ax5.plot(df['timestamp'], df['servo3'], label='Servo 3(LeftBack)')
    ax5.plot(df['timestamp'], df['servo4'], label='Servo 4(LeftTop)')
    ax5.set_title('Servo Output')
    ax5.set_ylabel('Servo value')
    ax5.legend()
    ax5.grid(True)
    ax5.xaxis.set_major_formatter(time_format)
    
    # 6. バッテリー、高度、温度のプロット
    ax6_1 = ax6
    ax6_2 = ax6.twinx()
    ax6_3 = ax6.twinx()
    
    color1, color2, color3 = '#1f77b4', '#ff7f0e', '#2ca02c'
    
    line1 = ax6_1.plot(df['timestamp'], df['battery'], color=color1, label='バッテリー')
    line2 = ax6_2.plot(df['timestamp'], df['altitude'], color=color2, label='高度')
    line3 = ax6_3.plot(df['timestamp'], df['temperature'], color=color3, label='温度')
    
    ax6_1.set_ylabel('Temperature (℃)', color=color1)
    ax6_2.set_ylabel('Altitude (m)', color=color2)
    ax6_3.set_ylabel('Temperature (℃)', color=color3)
    ax6_3.spines['right'].set_position(('outward', 60))
    
    # 凡例の結合
    lines = line1 + line2 + line3
    labels = [l.get_label() for l in lines]
    ax6.legend(lines, labels, loc='upper right')
    ax6.grid(True)
    ax6.xaxis.set_major_formatter(time_format)
    
    # グラフのレイアウト調整
    fig.tight_layout()
    
    # GUI表示の場合は保存しない
    if fig is None:
        plt.savefig('flight_log.png', dpi=300, bbox_inches='tight')
        plt.show()

def split_data_by_sequence(df):
    """シーケンス番号の連続性で飛行セッションを分割"""
    sessions = []
    current_session = []
    last_seqno = -1
    
    # データを時間順にソート
    df = df.sort_values(['timestamp', 'seqno'])
    
    for _, row in df.iterrows():
        if row['seqno'] < last_seqno:  # シーケンス番号が減少したら新しいセッション
            if current_session:
                session_df = pd.DataFrame(current_session).reset_index(drop=True)
                sessions.append(session_df)
            current_session = []
        current_session.append(row)
        last_seqno = row['seqno']
    
    if current_session:
        session_df = pd.DataFrame(current_session).reset_index(drop=True)
        sessions.append(session_df)
    
    return sessions

def analyze_session(df, session_num):
    """セッションの基本情報を表示"""
    duration = df['timestamp'].iloc[-1] - df['timestamp'].iloc[0]
    print(f"\nセッション {session_num} の概要:")
    print(f"開始時刻: {df['timestamp'].iloc[0].strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"終了時刻: {df['timestamp'].iloc[-1].strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"継続時間: {duration.total_seconds():.1f}秒")
    print(f"データ点数: {len(df)}")
    print(f"シーケンス番号範囲: {df['seqno'].min()} - {df['seqno'].max()}")
    print("-" * 50)

def select_session(sessions):
    """セッションの一覧を表示し、ユーザーに選択させる"""
    print("\n利用可能なセッション:")
    for i, session in enumerate(sessions, 1):
        duration = session['timestamp'].iloc[-1] - session['timestamp'].iloc[0]
        print(f"\n[{i}] セッション {i}")
        print(f"    開始: {session['timestamp'].iloc[0].strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"    時間: {duration.total_seconds():.1f}秒")
        print(f"    データ数: {len(session)}")

    while True:
        try:
            choice = int(input("\n分析するセッション番号を入力してください（1-{}）: ".format(len(sessions))))
            if 1 <= choice <= len(sessions):
                return choice, sessions[choice-1]  # インデックスとセッションの両方を返す
            else:
                print("有効な番号を入力してください")
        except ValueError:
            print("数字を入力してください")

def plot_imu_pid_pattern(df, fig=None):
    """IMUとPIDのパターンをプロット"""
    if fig is None:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(15, 12))
    else:
        fig.clear()
        ax1, ax2 = fig.subplots(2, 1)
    
    time_format = mdates.DateFormatter('%H:%M:%S')
    
    # IMU角度のプロット
    ax1.plot(df['timestamp'], df['imu_roll'], label='Roll')
    ax1.plot(df['timestamp'], df['imu_pitch'], label='Pitch')
    ax1.plot(df['timestamp'], df['imu_yaw'], label='Yaw')
    ax1.set_title('IMU Angle')
    ax1.set_ylabel('Angle (deg)')
    ax1.legend()
    ax1.grid(True)
    ax1.xaxis.set_major_formatter(time_format)
    
    # PID出力のプロット
    ax2.plot(df['timestamp'], df['pid_roll'], label='Roll')
    ax2.plot(df['timestamp'], df['pid_pitch'], label='Pitch')
    ax2.plot(df['timestamp'], df['pid_yaw'], label='Yaw')
    ax2.set_title('PID Output')
    ax2.set_ylabel('Output value')
    ax2.legend()
    ax2.grid(True)
    ax2.xaxis.set_major_formatter(time_format)
    
    fig.tight_layout()

def plot_servo_pid_pattern(df, fig=None):
    """サーボとPIDのパターンをプロット"""
    if fig is None:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(15, 12))
    else:
        fig.clear()
        ax1, ax2 = fig.subplots(2, 1)
    
    time_format = mdates.DateFormatter('%H:%M:%S')
    
    # サーボ値のプロット
    ax1.plot(df['timestamp'], df['servo1'], label='Servo 1(RightBack)')
    ax1.plot(df['timestamp'], df['servo2'], label='Servo 2(RightTop)')
    ax1.plot(df['timestamp'], df['servo3'], label='Servo 3(LeftBack)')
    ax1.plot(df['timestamp'], df['servo4'], label='Servo 4(LeftTop)')
    ax1.set_title('Servo Output')
    ax1.set_ylabel('Servo value')
    ax1.legend()
    ax1.grid(True)
    ax1.xaxis.set_major_formatter(time_format)
    
    # PID出力のプロット
    ax2.plot(df['timestamp'], df['pid_roll'], label='Roll')
    ax2.plot(df['timestamp'], df['pid_pitch'], label='Pitch')
    ax2.plot(df['timestamp'], df['pid_yaw'], label='Yaw')
    ax2.set_title('PID Output')
    ax2.set_ylabel('Output value')
    ax2.legend()
    ax2.grid(True)
    ax2.xaxis.set_major_formatter(time_format)
    
    fig.tight_layout()

def plot_servo_battery_pattern(df, fig=None):
    """サーボとバッテリー、温度のパターンをプロット"""
    if fig is None:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(15, 12))
    else:
        fig.clear()
        ax1, ax2 = fig.subplots(2, 1)
    
    time_format = mdates.DateFormatter('%H:%M:%S')
    
    # サーボ値のプロット
    ax1.plot(df['timestamp'], df['servo1'], label='Servo 1(RightBack)')
    ax1.plot(df['timestamp'], df['servo2'], label='Servo 2(RightTop)')
    ax1.plot(df['timestamp'], df['servo3'], label='Servo 3(LeftBack)')
    ax1.plot(df['timestamp'], df['servo4'], label='Servo 4(LeftTop)')
    ax1.set_title('Servo Output')
    ax1.set_ylabel('Servo value')
    ax1.legend()
    ax1.grid(True)
    ax1.xaxis.set_major_formatter(time_format)
    
    # バッテリーと温度のプロット
    ax2_1 = ax2
    ax2_2 = ax2.twinx()
    
    color1, color2 = '#1f77b4', '#ff7f0e'
    
    line1 = ax2_1.plot(df['timestamp'], df['battery'], color=color1, label='Battery')
    line2 = ax2_2.plot(df['timestamp'], df['temperature'], color=color2, label='Temperature')
    
    ax2_1.set_ylabel('Voltage (V)', color=color1)
    ax2_2.set_ylabel('Temperature (℃)', color=color2)
    
    lines = line1 + line2
    labels = [l.get_label() for l in lines]
    ax2.legend(lines, labels, loc='upper right')
    ax2.grid(True)
    ax2.xaxis.set_major_formatter(time_format)
    
    fig.tight_layout()

def plot_accel_pid_pattern(df, fig=None):
    """加速度とPIDのパターンをプロット"""
    if fig is None:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(15, 12))
    else:
        fig.clear()
        ax1, ax2 = fig.subplots(2, 1)
    
    time_format = mdates.DateFormatter('%H:%M:%S')
    
    # 加速度データのプロット
    ax1.plot(df['timestamp'], df['acc_x'], label='X')
    ax1.plot(df['timestamp'], df['acc_y'], label='Y')
    ax1.plot(df['timestamp'], df['acc_z'], label='Z')
    ax1.set_title('Accelerometer')
    ax1.set_ylabel('Acceleration (m/s²)')
    ax1.legend()
    ax1.grid(True)
    ax1.xaxis.set_major_formatter(time_format)
    
    # PID出力のプロット
    ax2.plot(df['timestamp'], df['pid_roll'], label='Roll')
    ax2.plot(df['timestamp'], df['pid_pitch'], label='Pitch')
    ax2.plot(df['timestamp'], df['pid_yaw'], label='Yaw')
    ax2.set_title('PID Output')
    ax2.set_ylabel('Output value')
    ax2.legend()
    ax2.grid(True)
    ax2.xaxis.set_major_formatter(time_format)
    
    fig.tight_layout()

def plot_gyro_pid_pattern(df, fig=None):
    """ジャイロとPIDのパターンをプロット"""
    if fig is None:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(15, 12))
    else:
        fig.clear()
        ax1, ax2 = fig.subplots(2, 1)
    
    time_format = mdates.DateFormatter('%H:%M:%S')
    
    # ジャイロデータのプロット
    ax1.plot(df['timestamp'], df['gyro_x'], label='X')
    ax1.plot(df['timestamp'], df['gyro_y'], label='Y')
    ax1.plot(df['timestamp'], df['gyro_z'], label='Z')
    ax1.set_title('Gyroscope')
    ax1.set_ylabel('Angular velocity (deg/s)')
    ax1.legend()
    ax1.grid(True)
    ax1.xaxis.set_major_formatter(time_format)
    
    # PID出力のプロット
    ax2.plot(df['timestamp'], df['pid_roll'], label='Roll')
    ax2.plot(df['timestamp'], df['pid_pitch'], label='Pitch')
    ax2.plot(df['timestamp'], df['pid_yaw'], label='Yaw')
    ax2.set_title('PID Output')
    ax2.set_ylabel('Output value')
    ax2.legend()
    ax2.grid(True)
    ax2.xaxis.set_major_formatter(time_format)
    
    fig.tight_layout()

class LogAnalyzerGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('ドローンログ分析ツール')
        self.setGeometry(100, 100, 1200, 800)
        
        # メインウィジェットとレイアウトの設定
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout(main_widget)
        
        # ファイル選択部分
        file_layout = QHBoxLayout()
        self.file_label = QLabel('ログファイル: 未選択')
        self.select_file_btn = QPushButton('ファイルを選択')
        self.select_file_btn.clicked.connect(self.select_file)
        file_layout.addWidget(self.file_label)
        file_layout.addWidget(self.select_file_btn)
        layout.addLayout(file_layout)
        
        # セッション選択部分
        session_layout = QHBoxLayout()
        self.session_label = QLabel('セッション:')
        self.session_combo = QComboBox()
        self.session_combo.currentIndexChanged.connect(self.on_session_changed)
        session_layout.addWidget(self.session_label)
        session_layout.addWidget(self.session_combo)
        layout.addLayout(session_layout)
        
        # 分析ボタン
        self.analyze_btn = QPushButton('分析を実行')
        self.analyze_btn.clicked.connect(self.run_analysis)
        self.analyze_btn.setEnabled(False)
        layout.addWidget(self.analyze_btn)
        
        # ステータスラベル
        self.status_label = QLabel('')
        layout.addWidget(self.status_label)
        
        # タブウィジェットの作成
        self.tab_widget = QTabWidget()
        layout.addWidget(self.tab_widget)
        
        # 各タブの作成
        self.tabs = []
        self.canvases = []
        self.figs = []
        
        # タブの作成
        tab_names = [
            'IMU vs. PID',
            'Servo vs. PID',
            'Servo vs. Battery, Temperature',
            'Accelerometer vs. PID',
            'Gyroscope vs. PID'
        ]
        
        plot_functions = [
            plot_imu_pid_pattern,
            plot_servo_pid_pattern,
            plot_servo_battery_pattern,
            plot_accel_pid_pattern,
            plot_gyro_pid_pattern
        ]
        
        for name, plot_func in zip(tab_names, plot_functions):
            tab = QWidget()
            tab_layout = QVBoxLayout(tab)
            
            # スクロール可能なグラフ表示エリア
            scroll = QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
            scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
            
            # グラフ表示用のウィジェット
            graph_widget = QWidget()
            graph_layout = QVBoxLayout(graph_widget)
            scroll.setWidget(graph_widget)
            tab_layout.addWidget(scroll)
            
            self.tabs.append(tab)
            self.canvases.append(None)
            self.figs.append(None)
            self.tab_widget.addTab(tab, name)
        
        self.df = None
        self.sessions = []
        
    def select_file(self):
        file_path, _ = QFileDialog.getOpenFileName(
            self, 'ログファイルを選択', '', 'CSV Files (*.csv)')
        
        if file_path:
            try:
                self.df = pd.read_csv(file_path, parse_dates=['timestamp'])
                self.sessions = split_data_by_sequence(self.df)
                
                self.file_label.setText(f'ログファイル: {Path(file_path).name}')
                self.session_combo.clear()
                for i, session in enumerate(self.sessions, 1):
                    duration = session['timestamp'].iloc[-1] - session['timestamp'].iloc[0]
                    self.session_combo.addItem(
                        f'セッション {i} ({duration.total_seconds():.1f}秒)')
                
                self.analyze_btn.setEnabled(True)
                self.status_label.setText('ファイルを読み込みました')
            except Exception as e:
                QMessageBox.critical(self, 'エラー', f'ファイルの読み込みに失敗しました: {str(e)}')
    
    def on_session_changed(self, index):
        if index >= 0 and self.sessions:
            session = self.sessions[index]
            duration = session['timestamp'].iloc[-1] - session['timestamp'].iloc[0]
            self.status_label.setText(
                f'セッション {index + 1} を選択: {duration.total_seconds():.1f}秒')
    
    def run_analysis(self):
        if not self.sessions or self.session_combo.currentIndex() < 0:
            return
            
        try:
            selected_session = self.sessions[self.session_combo.currentIndex()]
            analyze_session(selected_session, self.session_combo.currentIndex() + 1)
            
            # 各タブのグラフを更新
            plot_functions = [
                plot_imu_pid_pattern,
                plot_servo_pid_pattern,
                plot_servo_battery_pattern,
                plot_accel_pid_pattern,
                plot_gyro_pid_pattern
            ]
            
            for i, (canvas, fig, plot_func) in enumerate(zip(self.canvases, self.figs, plot_functions)):
                # 既存のグラフをクリア
                if canvas:
                    self.tabs[i].findChild(QScrollArea).widget().layout().removeWidget(canvas)
                    canvas.deleteLater()
                
                # 新しいグラフを作成
                self.figs[i] = plt.figure(figsize=(15, 12))
                plot_func(selected_session, self.figs[i])
                self.canvases[i] = FigureCanvas(self.figs[i])
                self.tabs[i].findChild(QScrollArea).widget().layout().addWidget(self.canvases[i])
            
            self.status_label.setText('分析が完了しました')
        except Exception as e:
            QMessageBox.critical(self, 'エラー', f'分析中にエラーが発生しました: {str(e)}')

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = LogAnalyzerGUI()
    window.show()
    sys.exit(app.exec_())