from motor_control.motor1 import MotorControllerupdo  # 控制上下馬達
from motor_control.motor2 import MotorControllerLR   # 控制左右馬達
import json
import threading
import os

# 讀取校準結果
# def load_calibration(file_path):
#     try:
#         with open(file_path, 'r') as f:
#             return json.load(f)
#     except FileNotFoundError:
#         print(f"[ERROR] Calibration file {file_path} not found.")
#         return None

def load_calibration(filename):
    base_path = os.path.dirname(os.path.abspath(__file__))  # 取得當前腳本所在的資料夾
    file_path = os.path.join(base_path, filename)  # 拼接 JSON 檔案的路徑
    try:
        with open(file_path, 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"[ERROR] Calibration file {file_path} not found.")
        return None

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
    :param up_motor: 上下馬達控制器
    :param lr_motor: 左右馬達控制器
    :param up_angle: 上下目標角度
    :param lr_angle: 左右目標角度
    :param up_calib: 上下馬達校準數據
    :param lr_calib: 左右馬達校準數據
    """
    # 讀取上下馬達的校準數據
    up_zero = up_calib["zero_encoder"]
    up_min_angle, up_max_angle = up_calib["down_limit_angle"], up_calib["up_limit_angle"]

    # 讀取左右馬達的校準數據
    lr_zero = lr_calib["zero_encoder"]
    lr_min_angle, lr_max_angle = lr_calib["right_limit_angle"], lr_calib["left_limit_angle"]

    # 檢查目標角度是否在範圍內
    if not (up_min_angle <= up_angle <= up_max_angle):
        print(f"[ERROR] Up-Down angle {up_angle} out of range ({up_min_angle} ~ {up_max_angle}).")
        return
    if not (lr_min_angle <= lr_angle <= lr_max_angle):
        print(f"[ERROR] Left-Right angle {lr_angle} out of range ({lr_min_angle} ~ {lr_max_angle}).")
        return

    # 創建執行緒，分別控制上下和左右馬達
    up_thread = threading.Thread(target=move_motor_thread, args=(up_motor, up_zero, up_angle, "Up-Down Motor"))
    lr_thread = threading.Thread(target=move_motor_thread, args=(lr_motor, lr_zero, lr_angle, "Left-Right Motor"))

    # 啟動執行緒
    up_thread.start()
    lr_thread.start()

    # 等待執行緒完成
    up_thread.join()
    lr_thread.join()

def main():
    # 加載校準數據
    # up_calib = load_calibration('/home/robot/Downloads/wheeltec_ros2_back/src/rs485_fire/src/calibration_updo.json')
    # lr_calib = load_calibration('/home/robot/Downloads/wheeltec_ros2_back/src/rs485_fire/src/calibration_LR.json')

    up_calib = load_calibration("calibration_updo.json")
    lr_calib = load_calibration("calibration_LR.json")

    if not up_calib or not lr_calib:
        print("[ERROR] Calibration data not available. Exiting program.")
        return

    # 初始化馬達控制器
    up_motor = MotorControllerupdo(port='/dev/updottyUSB', baudrate=9600, unit=0x00)
    lr_motor = MotorControllerLR(port='/dev/LRttyUSB', baudrate=9600, unit=0x01)

    # 連接上下馬達
    if not up_motor.connect():
        print("Cannot connect to Up-Down Motor.")
        return
    print("Up-Down Motor connected.")

    if not lr_motor.connect():
        print("Cannot connect to Left-Right Motor.")
        return
    print("Left-Right Motor connected.")

    # 提示用戶輸入目標角度
    while True:
        user_input = input("Enter target angles (up_angle, lr_angle) or 'q' to quit: ").strip()
        if user_input.lower() == 'q':
            print("Exiting program.")
            break

        try:
            # 解析用戶輸入的角度
            up_angle, lr_angle = map(float, user_input.split(','))
            move_to_angles(up_motor, lr_motor, up_angle, lr_angle, up_calib, lr_calib)
        except ValueError:
            print("[ERROR] Invalid input. Please enter angles in the format: up_angle,lr_angle")
        except Exception as e:
            print(f"[ERROR] Unexpected error: {e}")

    # 關閉連接
    up_motor.close()
    lr_motor.close()
    print("Connections closed.")

if __name__ == "__main__":
    main()