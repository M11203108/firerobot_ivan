from motor_control.motor1 import MotorControllerupdo  # 控制上下馬達
from motor_control.motor2 import MotorControllerLR   # 控制左右馬達
import json
import threading
import os
import csv

# === 載入校準檔 ===
def load_calibration(filename):
    base_path = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(base_path, filename)
    try:
        with open(file_path, 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"[ERROR] Calibration file {file_path} not found.")
        return None

# === 載入 CSV 軌跡檔 ===
def load_trajectory_from_csv(filename="trajectory_angles.csv"):
    base_path = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(base_path, filename)
    trajectory = []
    try:
        with open(file_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                pitch = float(row['pitch'])
                yaw = float(row['yaw'])
                trajectory.append((pitch, yaw))
        print(f"[INFO] Loaded {len(trajectory)} points from {file_path}")
    except FileNotFoundError:
        print(f"[ERROR] Trajectory file {file_path} not found.")
    return trajectory

# === 分段 ===
def segment_trajectory(trajectory_list):
    segments = []
    current_pitch = None
    current_segment = []

    for pitch, yaw in trajectory_list:
        if pitch != current_pitch:
            if current_segment:
                segments.append((current_pitch, current_segment))
            current_segment = []
            current_pitch = pitch
        current_segment.append(yaw)

    if current_segment:
        segments.append((current_pitch, current_segment))

    return segments

# === 馬達移動 ===
def move_motor_thread(motor, zero_encoder, target_angle, name):
    try:
        print(f"{name} moving to {target_angle:.2f} degrees...")
        motor.move_to_angle(zero_encoder, target_angle)
        print(f"{name} reached {target_angle:.2f} degrees.")
    except Exception as e:
        print(f"[ERROR] {name} failed to move: {e}")

# === 主程式 ===
def main():
    # 載入校準資料
    up_calib = load_calibration("calibration_updo.json")
    lr_calib = load_calibration("calibration_LR.json")

    if not up_calib or not lr_calib:
        print("[ERROR] Calibration data not available. Exiting program.")
        return

    # 初始化馬達控制器
    up_motor = MotorControllerupdo(port='/dev/updottyUSB', baudrate=9600, unit=0x00)
    lr_motor = MotorControllerLR(port='/dev/LRttyUSB', baudrate=9600, unit=0x01)

    if not up_motor.connect():
        print("Cannot connect to Up-Down Motor.")
        return
    print("Up-Down Motor connected.")

    if not lr_motor.connect():
        print("Cannot connect to Left-Right Motor.")
        return
    print("Left-Right Motor connected.")

    # 載入 CSV 軌跡
    trajectory_list = load_trajectory_from_csv("trajectories/trajectory_angles.csv")

    if not trajectory_list:
        print("[ERROR] No trajectory data found. Exiting.")
    else:
        # 分段處理
        segments = segment_trajectory(trajectory_list)

        up_zero = up_calib["zero_encoder"]
        lr_zero = lr_calib["zero_encoder"]

        up_min_angle, up_max_angle = up_calib["down_limit_angle"], up_calib["up_limit_angle"]
        lr_min_angle, lr_max_angle = lr_calib["right_limit_angle"], lr_calib["left_limit_angle"]

        print("\n🔥 開始自動掃描。輸入 'q' 然後 Enter 可隨時中止。\n")

        reverse = False  # 初始是正向

        while True:
            segment_iter = segments if not reverse else reversed(segments)

            for idx, (pitch, yaws) in enumerate(segment_iter):
                hardware_pitch = 26.0 + pitch

                print(f"\n[Segment {idx+1}] Pitch: {pitch:.2f}° (hardware: {hardware_pitch:.2f}°) ({len(yaws)} Yaw Points)")

                # 檢查範圍（仰角）
                if not (up_min_angle <= hardware_pitch <= up_max_angle):
                    print(f"[ERROR] Up-Down angle {hardware_pitch:.2f} out of range ({up_min_angle} ~ {up_max_angle}). Skipping.")
                    continue

                move_motor_thread(up_motor, up_zero, hardware_pitch, "Up-Down Motor")

                # 掃描 Yaw
                min_yaw = min(yaws)
                max_yaw = max(yaws)

                if not (lr_min_angle <= min_yaw <= lr_max_angle):
                    print(f"[ERROR] Left-Right angle {min_yaw:.2f} out of range ({lr_min_angle} ~ {lr_max_angle}). Skipping.")
                    continue
                if not (lr_min_angle <= max_yaw <= lr_max_angle):
                    print(f"[ERROR] Left-Right angle {max_yaw:.2f} out of range ({lr_min_angle} ~ {lr_max_angle}). Skipping.")
                    continue

                if not reverse:
                    print(f"Scanning Yaw from {min_yaw:.2f}° to {max_yaw:.2f}°")
                    move_motor_thread(lr_motor, lr_zero, min_yaw, "Left-Right Motor")
                    move_motor_thread(lr_motor, lr_zero, max_yaw, "Left-Right Motor")
                else:
                    print(f"Scanning Yaw from {max_yaw:.2f}° back to {min_yaw:.2f}°")
                    move_motor_thread(lr_motor, lr_zero, max_yaw, "Left-Right Motor")
                    move_motor_thread(lr_motor, lr_zero, min_yaw, "Left-Right Motor")

            # 每輪結束，反向方向
            reverse = not reverse

            # 判斷是否停止
            user_input = input("\n按 Enter 繼續掃描，或輸入 'q' + Enter 來停止: ").strip().lower()
            if user_input == 'q':
                print("🚨 停止自動掃描。")
                break

    # 關閉連線
    up_motor.close()
    lr_motor.close()
    print("Connections closed.")

if __name__ == "__main__":
    main()

