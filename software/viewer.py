import tkinter as tk
from OpenGL import GL, GLU
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np
import win32gui
import win32api
import win32con
import ctypes
import numpy as np
from scipy.spatial.transform import Rotation

# Win32 定数の定義
PFD_DRAW_TO_WINDOW = 0x00000004
PFD_SUPPORT_OPENGL = 0x00000020
PFD_DOUBLEBUFFER = 0x00000001
PFD_TYPE_RGBA = 0
PFD_MAIN_PLANE = 0

# GDI32関数のロード
gdi32 = ctypes.WinDLL('gdi32')
ChoosePixelFormat = gdi32.ChoosePixelFormat
SetPixelFormat = gdi32.SetPixelFormat
SwapBuffers = gdi32.SwapBuffers  # SwapBuffers関数を追加
class PIXELFORMATDESCRIPTOR(ctypes.Structure):
    _fields_ = [
        ('nSize', ctypes.c_ushort),
        ('nVersion', ctypes.c_ushort),
        ('dwFlags', ctypes.c_ulong),
        ('iPixelType', ctypes.c_ubyte),
        ('cColorBits', ctypes.c_ubyte),
        ('cRedBits', ctypes.c_ubyte),
        ('cRedShift', ctypes.c_ubyte),
        ('cGreenBits', ctypes.c_ubyte),
        ('cGreenShift', ctypes.c_ubyte),
        ('cBlueBits', ctypes.c_ubyte),
        ('cBlueShift', ctypes.c_ubyte),
        ('cAlphaBits', ctypes.c_ubyte),
        ('cAlphaShift', ctypes.c_ubyte),
        ('cAccumBits', ctypes.c_ubyte),
        ('cAccumRedBits', ctypes.c_ubyte),
        ('cAccumGreenBits', ctypes.c_ubyte),
        ('cAccumBlueBits', ctypes.c_ubyte),
        ('cAccumAlphaBits', ctypes.c_ubyte),
        ('cDepthBits', ctypes.c_ubyte),
        ('cStencilBits', ctypes.c_ubyte),
        ('cAuxBuffers', ctypes.c_ubyte),
        ('iLayerType', ctypes.c_ubyte),
        ('bReserved', ctypes.c_ubyte),
        ('dwLayerMask', ctypes.c_ulong),
        ('dwVisibleMask', ctypes.c_ulong),
        ('dwDamageMask', ctypes.c_ulong),
    ]
class DroneViewer(tk.Canvas):
    def __init__(self, master=None, **kw):
        super().__init__(master, **kw)
        
        self.width = 60
        self.height = 450
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        # アニメーション用の目標値を追加
        self.target_roll = 0
        self.target_pitch = 0
        self.target_yaw = 0
        
        # アニメーション用のパラメータ
        self.animation_speed = 0.2  # 補間速度（0-1の値、1が最速）
        self.animation_active = False
        self.animation_frame_time = 16  # ミリ秒単位（約60FPS）
        
        self.hrc = None
        self.hdc = None
        
        # 最小サイズを設定
        self.configure(width=400, height=400)
        
        # イベントバインド
        self.bind('<Configure>', self.resize)
        self.bind('<Map>', self.on_map)
        self.bind('<Destroy>', self.cleanup)
        
        self.initialized = False
        self.init_attempts = 0
        self.max_init_attempts = 5
        self.init_delay = 1000  # 初期遅延時間（ミリ秒）

        # ウィンドウが表示されるまで待機
        self.update_idletasks()
        self.after(self.init_delay, self.initialize_gl)
    
    def resize(self, event):
        """ウィンドウサイズが変更されたときに呼ばれる"""
        if event.width > 1 and event.height > 1:
            self.width = event.width
            self.height = event.height
            if self.initialized:
                try:
                    from OpenGL import WGL
                    WGL.wglMakeCurrent(self.hdc, self.hrc)
                    
                    glViewport(0, 0, self.width, self.height)
                    glMatrixMode(GL_PROJECTION)
                    glLoadIdentity()
                    gluPerspective(45, float(self.width)/float(self.height), 0.1, 50.0)
                    glMatrixMode(GL_MODELVIEW)
                    
                    self.draw_drone()
                except Exception as e:
                    print(f"リサイズエラー: {str(e)}")

    def cleanup(self, event=None):
        """リソースのクリーンアップ"""
        if self.initialized:
            try:
                from OpenGL import WGL
                if self.hrc:
                    WGL.wglMakeCurrent(None, None)
                    WGL.wglDeleteContext(self.hrc)
                    self.hrc = None
                if self.hdc:
                    win32gui.ReleaseDC(self.winfo_id(), self.hdc)
                    self.hdc = None
            except Exception as e:
                print(f"Cleanup error: {str(e)}")
        self.initialized = False

    def setup_pixel_format(self):
        """ピクセルフォーマットの設定"""
        try:
            from OpenGL import WGL
            
            # 既存のコンテキストをクリーンアップ
            if self.hrc:
                WGL.wglMakeCurrent(None, None)
                WGL.wglDeleteContext(self.hrc)
                self.hrc = None
            if self.hdc:
                win32gui.ReleaseDC(self.winfo_id(), self.hdc)
                self.hdc = None

            # デバイスコンテキストの取得
            hwnd = self.winfo_id()
            self.hdc = win32gui.GetDC(hwnd)
            if not self.hdc:
                raise Exception(f"GetDC failed: {win32api.GetLastError()}")

            # より基本的なピクセルフォーマットの設定
            pfd = PIXELFORMATDESCRIPTOR()
            pfd.nSize = ctypes.sizeof(PIXELFORMATDESCRIPTOR)
            pfd.nVersion = 1
            pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER
            pfd.iPixelType = PFD_TYPE_RGBA
            pfd.cColorBits = 24  # 24ビットカラーに戻す
            pfd.cDepthBits = 16  # 16ビット深度バッファ
            pfd.iLayerType = PFD_MAIN_PLANE

            # ピクセルフォーマットの選択と設定
            pixel_format = ChoosePixelFormat(self.hdc, ctypes.byref(pfd))
            if not pixel_format:
                raise Exception(f"ChoosePixelFormat failed: {win32api.GetLastError()}")

            if not SetPixelFormat(self.hdc, pixel_format, ctypes.byref(pfd)):
                raise Exception(f"SetPixelFormat failed: {win32api.GetLastError()}")

            # OpenGL 2.1コンテキストの作成を試みる
            try:
                # WGL_ARB_create_context拡張をチェック
                if hasattr(WGL, 'wglCreateContextAttribsARB'):
                    # OpenGL 2.1の属性を設定
                    attribs = [
                        WGL.CONTEXT_MAJOR_VERSION_ARB, 2,
                        WGL.CONTEXT_MINOR_VERSION_ARB, 1,
                        0
                    ]
                    self.hrc = WGL.wglCreateContextAttribsARB(self.hdc, None, attribs)
                else:
                    # 従来の方法でコンテキストを作成
                    self.hrc = WGL.wglCreateContext(self.hdc)
            except:
                # エラーが発生した場合は従来の方法でコンテキストを作成
                self.hrc = WGL.wglCreateContext(self.hdc)

            if not self.hrc:
                error = win32api.GetLastError()
                raise Exception(f"wglCreateContext failed: {error}")

            # コンテキストをカレントに設定
            if not WGL.wglMakeCurrent(self.hdc, self.hrc):
                error = win32api.GetLastError()
                raise Exception(f"wglMakeCurrent failed: {error}")

            return True
            
        except Exception as e:
            print(f"ピクセルフォーマット設定エラー: {str(e)}")
            self.cleanup()
            return False
        
    def initialize_gl(self):
        """OpenGLの初期化を実行"""
        if not self.initialized and self.winfo_ismapped():
            if self.init_attempts >= self.max_init_attempts:
                print("OpenGL初期化の最大試行回数に達しました")
                return

            try:
                if self.setup_pixel_format():
                    if self.init_gl():
                        self.initialized = True
                        self.draw_drone()
                        return
                    else:
                        print("OpenGL initialization failed")
                else:
                    print("Pixel format setup failed")
            except Exception as e:
                print(f"初期化エラー: {str(e)}")

            self.init_attempts += 1
            # 再試行（待機時間を徐々に増やす）
            self.init_delay += 500
            self.after(self.init_delay, self.initialize_gl)
                
    def init_gl(self):
        """OpenGLの初期化"""
        try:
            # 現在のコンテキストが有効か確認
            from OpenGL import WGL
            if not WGL.wglGetCurrentContext():
                WGL.wglMakeCurrent(self.hdc, self.hrc)
            
            # バッファのクリア色を設定
            glClearColor(0.2, 0.2, 0.2, 1.0)
            
            # 深度テストを有効化
            glEnable(GL_DEPTH_TEST)
            glDepthFunc(GL_LESS)
            
            # ライティングを有効化
            glEnable(GL_LIGHTING)
            glEnable(GL_LIGHT0)
            glEnable(GL_COLOR_MATERIAL)
            
            # 光源の設定
            glLight(GL_LIGHT0, GL_POSITION, (5.0, 5.0, 5.0, 1.0))
            
            # 投影行列の設定
            glMatrixMode(GL_PROJECTION)
            glLoadIdentity()
            gluPerspective(45, float(self.width)/float(self.height), 0.1, 100.0)
            
            # モデルビュー行列の設定
            glMatrixMode(GL_MODELVIEW)
            glLoadIdentity()
            
            return True
            
        except Exception as e:
            print(f"OpenGL初期化エラー: {str(e)}")
            return False

    def on_map(self, event):
        """ウィンドウがマップされたときに呼ばれる"""
        if not self.initialized:
            self.initialize_gl()
        
    def update_attitude(self, roll, pitch, yaw):
        """姿勢の更新（アニメーション付き）"""
        self.target_roll = roll
        self.target_pitch = pitch
        self.target_yaw = yaw
        
        if not self.animation_active:
            self.animation_active = True
            self.animate_attitude()

    def animate_attitude(self):
        """姿勢のアニメーション処理"""
        if not self.initialized:
            self.animation_active = False
            return
            
        # 現在値と目標値の差分を計算
        roll_diff = self.target_roll - self.roll
        pitch_diff = self.target_pitch - self.pitch
        yaw_diff = self.target_yaw - self.yaw
        
        # 値の更新
        self.roll += roll_diff * self.animation_speed
        self.pitch += pitch_diff * self.animation_speed
        self.yaw += yaw_diff * self.animation_speed
        
        # 描画更新
        self.draw_drone()
        
        # アニメーションの継続判定
        if (abs(roll_diff) > 0.01 or 
            abs(pitch_diff) > 0.01 or 
            abs(yaw_diff) > 0.01):
            self.after(self.animation_frame_time, self.animate_attitude)
        else:
            self.animation_active = False
    
    def _draw_drone_body(self):
        """ドローンの本体を描画"""
        # アーム長さとプロペラサイズの定義
        arm_length = 1.0
        arm_width = 0.1
        prop_size = 0.3
        body_size = 0.4
        
        # メインボディ（中央の箱）を描画
        glPushMatrix()
        glColor3f(0.3, 0.3, 0.3)  # ダークグレー
        
        # 中央ボディ
        glScale(body_size, 0.15, body_size)
        self._draw_cube()
        glPopMatrix()
        
        # 4つのアームを描画
        arm_angles = [0, 90, 180, 270] 
        for i, angle in enumerate(arm_angles):
            glPushMatrix()
            angle = i * 90  # 90度ずつ回転
            # glRotatef(angle, 0, 1, 0)
            glRotatef(angle - 90, 0, 1, 0)
            
            # アーム
            glPushMatrix()
            glTranslatef(arm_length/2, 0, 0)
            glScale(arm_length, 0.05, arm_width)
            if i == 1 or i == 0:  # 前方のアームは赤色
                glColor3f(1.0, 0.0, 0.0)  # 赤
            else:
                glColor3f(0.4, 0.4, 0.4)  # グレー
            self._draw_cube()
            glPopMatrix()
            
            # # プロペラのモーターマウント
            # glPushMatrix()
            # glTranslatef(arm_length, 0, 0)
            # glColor3f(1.0, 0.0, 0.0)  # 赤
            # glScale(0.2, 0.2, 0.2)
            # self._draw_cube()
            # glPopMatrix()
            
            # プロペラ
            glPushMatrix()
            glTranslatef(arm_length, 0.1, 0)
            if i % 2 == 0:  # 対角のプロペラは同じ色
                glColor3f(0.8, 0.8, 0.0)  # 黄色
            else:
                glColor3f(0.0, 0.8, 0.8)  # シアン
            self._draw_propeller(prop_size)
            glPopMatrix()
            
            glPopMatrix()
        
        # 前方向を示すマーカー
        glPushMatrix()
        glTranslatef(body_size, 0, 0)
        glColor3f(1.0, 0.0, 0.0)  # 赤
        glScale(0.2, 0.2, 0.2)
        self._draw_arrow()
        glPopMatrix()

    def _draw_cube(self):
        """単位立方体を描画"""
        glBegin(GL_QUADS)
        # 上面
        glNormal3f(0.0, 1.0, 0.0)
        glVertex3f(-0.5, 0.5, 0.5)
        glVertex3f(0.5, 0.5, 0.5)
        glVertex3f(0.5, 0.5, -0.5)
        glVertex3f(-0.5, 0.5, -0.5)
        
        # 下面
        glNormal3f(0.0, -1.0, 0.0)
        glVertex3f(-0.5, -0.5, -0.5)
        glVertex3f(0.5, -0.5, -0.5)
        glVertex3f(0.5, -0.5, 0.5)
        glVertex3f(-0.5, -0.5, 0.5)
        
        # 前面
        glNormal3f(0.0, 0.0, 1.0)
        glVertex3f(-0.5, -0.5, 0.5)
        glVertex3f(0.5, -0.5, 0.5)
        glVertex3f(0.5, 0.5, 0.5)
        glVertex3f(-0.5, 0.5, 0.5)
        
        # 背面
        glNormal3f(0.0, 0.0, -1.0)
        glVertex3f(-0.5, 0.5, -0.5)
        glVertex3f(0.5, 0.5, -0.5)
        glVertex3f(0.5, -0.5, -0.5)
        glVertex3f(-0.5, -0.5, -0.5)
        
        # 右面
        glNormal3f(1.0, 0.0, 0.0)
        glVertex3f(0.5, -0.5, 0.5)
        glVertex3f(0.5, -0.5, -0.5)
        glVertex3f(0.5, 0.5, -0.5)
        glVertex3f(0.5, 0.5, 0.5)
        
        # 左面
        glNormal3f(-1.0, 0.0, 0.0)
        glVertex3f(-0.5, -0.5, -0.5)
        glVertex3f(-0.5, -0.5, 0.5)
        glVertex3f(-0.5, 0.5, 0.5)
        glVertex3f(-0.5, 0.5, -0.5)
        glEnd()

    def _draw_propeller(self, size):
        """プロペラを描画"""
        glBegin(GL_TRIANGLES)
        # プロペラの羽根1
        glVertex3f(0, 0, 0)
        glVertex3f(size, 0, size/4)
        glVertex3f(size, 0, -size/4)
        
        # プロペラの羽根2
        glVertex3f(0, 0, 0)
        glVertex3f(-size, 0, size/4)
        glVertex3f(-size, 0, -size/4)
        glEnd()

    def _draw_arrow(self):
        """前方向を示す矢印を描画"""
        glBegin(GL_TRIANGLES)
        # 矢印の頭
        glVertex3f(1.0, 0, 0)    # 先端
        glVertex3f(0.5, 0.5, 0)  # 後端上
        glVertex3f(0.5, -0.5, 0) # 後端下
        glEnd()

    
    def _draw_axes(self):
        """座標軸の描画"""
        glBegin(GL_LINES)
        # X軸（赤）- 45度回転
        glColor3f(1.0, 0.0, 0.0)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(2.0 * np.cos(np.pi/4), 2.0 * np.sin(np.pi/4), 0.0)
        # Y軸（緑）- 45度回転
        glColor3f(0.0, 1.0, 0.0)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(-2.0 * np.sin(np.pi/4), 2.0 * np.cos(np.pi/4), 0.0)
        # Z軸（青）
        glColor3f(0.0, 0.0, 1.0)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(0.0, 0.0, 2.0)
        glEnd()
    
    def draw_drone(self):
        """ドローンの描画"""
        if not self.initialized:
            return
            
        try:
            from OpenGL import WGL
            WGL.wglMakeCurrent(self.hdc, self.hrc)
            
            # バッファをクリア
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            glLoadIdentity()
            
            # カメラの位置設定
            gluLookAt(5.0, 5.0, 5.0,  # カメラの位置
                     0.0, 0.0, 0.0,   # 注視点
                     0.0, 1.0, 0.0)   # 上方向ベクトル
            
            # 角度を正規化（-180から180の範囲に収める）
            roll = -(((self.roll + 180) % 360) - 180)
            pitch = ((self.pitch + 180) % 360) - 180
            yaw = ((self.yaw + 180) % 360) - 180
            
            # 各軸周りの回転を作成
            r_roll = Rotation.from_rotvec(np.array([0, 0, 1]) * np.radians(roll))
            r_pitch = Rotation.from_rotvec(np.array([1, 0, 0]) * np.radians(pitch))
            r_yaw = Rotation.from_rotvec(np.array([0, 1, 0]) * np.radians(yaw))
            
            # 回転を合成
            r = r_yaw * r_pitch * r_roll
            
            # 回転行列を取得してOpenGL形式に変換
            rotation_matrix = r.as_matrix()
            gl_matrix = np.eye(4)
            gl_matrix[:3, :3] = rotation_matrix
            
            # 回転行列を適用
            glMultMatrixf(gl_matrix.T.flatten().astype(np.float32))
            
            # モデルの描画
            self._draw_drone_body()
            self._draw_axes()
            
            # バッファの入れ替え
            SwapBuffers(self.hdc)
            
        except Exception as e:
            print(f"描画エラー: {str(e)}")
    
    def update_attitude(self, roll, pitch, yaw):
        """姿勢の更新"""
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        if self.initialized:
            self.draw_drone()