import sys
import os
from motor_control.motor1 import MotorControllerupdo  # 控制上下馬達
from motor_control.motor2 import MotorControllerLR   # 控制左右馬達
sys.path.append(os.path.abspath('/home/robot/wheeltec_ros2/src/IR_camera/src/'))
from test_find_point_real import SprayFindPointReal       # 熱源數據獲取
import json
import threading
import time

def generate_z_trajectory(lr_angle, up_angle, lr_range=10, up_step=10, up_min_angle=0.42):
    """
    生成 Z 字形運動軌跡（從上到下）。
    :param lr_angle: 火源的左右角度
    :param up_angle: 火源的上下角度
    :param lr_range: 左右角度的偏移範圍（默認為 ±10°）
    :param up_step: 上下角度每次下降的步長（默認為 5°）
    :param up_min_angle: 上下角度的最低值（默認為 0.42°）
    :return: Z 字形軌跡列表，格式 [(lr_angle, up_angle), ...]
    """
    trajectory = []

    # 計算左右角度範圍
    lr_min = lr_angle - lr_range
    lr_max = lr_angle + lr_range

    # 初始化當前上下角度
    current_up_angle = up_angle

    while current_up_angle >= up_min_angle:
        # 左到右
        trajectory.append((lr_min, current_up_angle))
        trajectory.append((lr_max, current_up_angle))

        # 更新上下角度（向下移動）
        current_up_angle -= up_step

    # 完成一個循環後，返回最高點
    trajectory.append((lr_min, up_angle))
    return trajectory

def z_motion(up_motor, lr_motor, up_angle, lr_angle, up_calib, lr_calib, duration=300):
    """
    控制馬達進行 Z 字形運動（從上往下）。
    :param up_motor: 上下馬達控制器
    :param lr_motor: 左右馬達控制器
    :param up_angle: 修正後的上下角度
    :param lr_angle: 左右角度
    :param up_calib: 上下馬達校準數據
    :param lr_calib: 左右馬達校準數據
    :param duration: 運動持續時間（秒），默認 60 秒
    """
    # 獲取零點編碼器數據
    up_zero = up_calib["zero_encoder"]
    lr_zero = lr_calib["zero_encoder"]

    # 生成 Z 字形運動軌跡
    trajectory = generate_z_trajectory(lr_angle, up_angle)

    start_time = time.time()
    while time.time() - start_time < duration:
        for lr, up in trajectory:
            print(f"Moving to: LR={lr:.2f}°, UP={up:.2f}°")
            move_to_angles(up_motor, lr_motor, up, lr, up_calib, lr_calib)

            # 檢查時間是否超過
            if time.time() - start_time >= duration:
                break
            
# 讀取校準結果
def load_calibration(file_path):
    try:
        with open(file_path, 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"[ERROR] Calibration file {file_path} not found.")
        return None

def calculate_up_angle(distance):
    """
    根據距離計算上下角度（簡單的距離到角度的映射，可根據需求調整公式）。
    :param distance: 火源與機器人之間的距離
    :return: 上下角度
    """
    max_distance = 5.0  # 假設最大距離為 5 米
    max_angle = 61.45   # 最大角度為 61.45 度
    min_angle = 0.0     # 最小角度為 0 度

    # 確保距離在範圍內
    if distance > max_distance:
        distance = max_distance
    elif distance < 0:
        distance = 0

    # 根據距離線性映射角度
    up_angle = min_angle + (max_angle - min_angle) * (distance / max_distance)
    return up_angle

def move_motor_thread(motor, zero_encoder, target_angle, name):
    """
    馬達移動執行緒函數。
    :param motor: MotorController 實例
    :param zero_encoder: 零點位置
    :param target_angle: 目標角度
    :param name: 馬達名稱（Up-Down 或 Left-Right）
    """
    try:
        print(f"{name} moving to {target_angle} degrees...")
        motor.move_to_angle(zero_encoder, target_angle)
        print(f"{name} reached {target_angle} degrees.")
    except Exception as e:
        print(f"[ERROR] {name} failed to move: {e}")

def move_to_angles(up_motor, lr_motor, up_angle, lr_angle, up_calib, lr_calib):
    """
    控制馬達同時移動到指定的上下角度和左右角度。
    """
    up_zero = up_calib["zero_encoder"]
    lr_zero = lr_calib["zero_encoder"]

    # 創建執行緒，分別控制上下和左右馬達
    up_thread = threading.Thread(target=move_motor_thread, args=(up_motor, up_zero, up_angle, "Up-Down Motor"))
    lr_thread = threading.Thread(target=move_motor_thread, args=(lr_motor, lr_zero, lr_angle, "Left-Right Motor"))

    # 啟動執行緒
    up_thread.start()
    lr_thread.start()

    # 等待執行緒完成
    up_thread.join()
    lr_thread.join()

def calculate_corrected_up_angle(target_angle, lower_limit_angle=0, upper_limit_angle=64.425, offset=26):
    """
    計算修正後的上下角度。
    :param target_angle: 使用者或自動模式計算的目標角度
    :param lower_limit_angle: 上下角度的最小限制（默認為 0 度）
    :param upper_limit_angle: 上下角度的最大限制（默認為 64.425 度）
    :param offset: 馬達 0 度水平偏移量（默認為 26 度）
    :return: 修正後的上下角度
    """
    # 加入偏移量修正
    corrected_angle = target_angle + offset

    # 限制角度範圍
    corrected_angle = max(lower_limit_angle, min(corrected_angle, upper_limit_angle))
    return corrected_angle

def move_to_angles(up_motor, lr_motor, up_angle, lr_angle, up_calib, lr_calib):
    """
    控制馬達同時移動到指定的上下角度和左右角度。
    :param up_motor: 上下馬達控制器
    :param lr_motor: 左右馬達控制器
    :param up_angle: 上下目標角度
    :param lr_angle: 左右目標角度
    :param up_calib: 上下馬達校準數據
    :param lr_calib: 左右馬達校準數據
    """
    up_zero = up_calib["zero_encoder"]
    lr_zero = lr_calib["zero_encoder"]

    # 計算修正後的上下角度
    up_corrected_angle = calculate_corrected_up_angle(up_angle)

    # 創建執行緒，分別控制上下和左右馬達
    up_thread = threading.Thread(target=move_motor_thread, args=(up_motor, up_zero, up_corrected_angle, "Up-Down Motor"))
    lr_thread = threading.Thread(target=move_motor_thread, args=(lr_motor, lr_zero, lr_angle, "Left-Right Motor"))

    # 啟動執行緒
    up_thread.start()
    lr_thread.start()

    # 等待執行緒完成
    up_thread.join()
    lr_thread.join()

def spray_fire(up_motor, lr_motor, up_calib, lr_calib):
    """
    自動噴水模式，根據熱源位置控制馬達。
    """
    spray_finder = SprayFindPointReal()  # 初始化熱源檢測類

    # 從 Memcached 獲取距離和角度
    distance, lr_angle, direction = spray_finder.main()

    # 根據距離計算上下角度
    up_angle = calculate_up_angle(distance)

    # 計算修正後的上下角度
    up_corrected_angle = calculate_corrected_up_angle(up_angle)

    print(f"火源位置 -> 距離: {distance:.2f}m, 左右角度: {lr_angle:.2f}°, 修正後上下角度: {up_corrected_angle:.2f}°")

    # 控制馬達移動到計算出的角度
    move_to_angles(up_motor, lr_motor, up_corrected_angle, lr_angle, up_calib, lr_calib)


def main():
    # 加載校準數據
    up_calib = load_calibration('/home/robot/wheeltec_ros2/src/rs485_fire/src/calibration_updo.json')
    lr_calib = load_calibration('/home/robot/wheeltec_ros2/src/rs485_fire/src/calibration_LR.json')

    if not up_calib or not lr_calib:
        print("[ERROR] Calibration data not available. Exiting program.")
        return

    # 初始化馬達控制器
    up_motor = MotorControllerupdo(port='/dev/updottyUSB', baudrate=9600, unit=0x00)
    lr_motor = MotorControllerLR(port='/dev/LRttyUSB', baudrate=9600, unit=0x01)

    # 連接馬達
    if not up_motor.connect():
        print("Cannot connect to Up-Down Motor.")
        return
    print("Up-Down Motor connected.")

    if not lr_motor.connect():
        print("Cannot connect to Left-Right Motor.")
        return
    print("Left-Right Motor connected.")

    # 提示模式選擇
    while True:
        mode = input("Enter 'manual' for manual mode, 'auto' for auto spray mode, 'z-motion' for Z-motion mode, or 'q' to quit: ").strip().lower()
        if mode == 'q':
            print("Exiting program.")
            break
        elif mode == 'manual':
            try:
                user_input = input("Enter target angles (up_angle, lr_angle): ").strip()
                up_angle, lr_angle = map(float, user_input.split(','))
                move_to_angles(up_motor, lr_motor, up_angle, lr_angle, up_calib, lr_calib)
            except ValueError:
                print("[ERROR] Invalid input. Please enter angles in the format: up_angle,lr_angle")
        elif mode == 'auto':
            spray_fire(up_motor, lr_motor, up_calib, lr_calib)
        elif mode == 'z-motion':
            # 獲取火源數據
            spray_finder = SprayFindPointReal()
            distance, lr_angle, direction = spray_finder.main()
            up_angle = calculate_up_angle(distance)

            # 計算修正後上下角度
            up_corrected_angle = calculate_corrected_up_angle(up_angle)

            print(f"Starting Z-motion with: LR={lr_angle:.2f}°, UP={up_corrected_angle:.2f}°")
            z_motion(up_motor, lr_motor, up_corrected_angle, lr_angle, up_calib, lr_calib)
        else:
            print("[ERROR] Invalid mode. Please enter 'manual', 'auto', 'z-motion', or 'q'.")

    # 關閉連接
    up_motor.close()
    lr_motor.close()
    print("Connections closed.")

if __name__ == "__main__":
    main()

