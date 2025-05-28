from pymodbus.client.sync import ModbusSerialClient as ModbusClient
import time
import struct
from pynput import keyboard

# 將浮點數轉換為16進位表示的字串
def float_to_hex(f):
    packed = struct.pack('!f', f)
    return packed.hex()

# 讀取編碼器的原始值（未轉換為圈數）
def read_encoder_raw(client, unit):
    position_high = client.read_holding_registers(0x0007, 1, unit=unit).registers[0]
    position_low = client.read_holding_registers(0x0008, 1, unit=unit).registers[0]
    encoder_value = (position_high << 16) | position_low
    if encoder_value & 0x80000000:  # 處理負值
        encoder_value = -(0xFFFFFFFF - encoder_value + 1)
    return encoder_value

# 控制馬達移動至目標圈數
def move_motor(client, unit, direction, target_circle, set_rpm):
    ieee_circle = float_to_hex(target_circle)
    ieee_circle_high = int('0x' + ieee_circle[0:4], 16)
    ieee_circle_low = int('0x' + ieee_circle[4:8], 16)

    ieee_754_rpm = float_to_hex(set_rpm)
    ieee_754_rpm_high = int('0x' + ieee_754_rpm[0:4], 16)
    ieee_754_rpm_low = int('0x' + ieee_754_rpm[4:8], 16)

    client.write_register(0x004F, 0x0001, unit=unit)  # 啟動馬達
    client.write_register(0x0053, ieee_754_rpm_high, unit=unit)  # 設定轉速高位
    client.write_register(0x0054, ieee_754_rpm_low, unit=unit)  # 設定轉速低位
    client.write_register(0x0050, direction, unit=unit)  # 設置方向
    client.write_register(0x0055, ieee_circle_high, unit=unit)  # 設置目標圈數高位
    client.write_register(0x0056, ieee_circle_low, unit=unit)  # 設置目標圈數低位
    print(f"Motor moving: direction={direction}, target_circle={target_circle:.4f}")

# 根據編碼器值計算目標圈數並移動到指定位置
def move_to_zero_position(client, unit, current_encoder, zero_encoder, set_rpm, resolution):
    print(f"Moving to zero position: {zero_encoder} from {current_encoder}")

    # 計算目標圈數
    position_diff = abs(zero_encoder - current_encoder)
    target_circle = position_diff / resolution
    direction = 0x0000

    # 移動馬達
    move_motor(client, unit, direction, target_circle, set_rpm)

    # 等待移動完成
    while True:
        current_encoder = read_encoder_raw(client, unit)
        print(f"Current encoder: {current_encoder}, Target encoder: {zero_encoder}")
        if abs(current_encoder - zero_encoder) < 100:  # 設定允許的誤差範圍
            break
        time.sleep(0.1)

    # 停止馬達
    client.write_register(0x004F, 0x0000, unit=unit)
    print("Motor stopped at zero position.")

# 執行歸零校準
def zero_calibration(client, unit, resolution):
    print("Starting zero calibration...")

    # 左極限
    print("Moving to left limit...")
    left_limit = None
    stable_count = 0
    previous_encoder = None
    while True:
        move_motor(client, unit, 0x0000, 1.0, 50)  # 嘗試大範圍移動
        current_encoder = read_encoder_raw(client, unit)
        print(f"Current encoder: {current_encoder}")

        if previous_encoder == current_encoder:
            stable_count += 1
        else:
            stable_count = 0

        previous_encoder = current_encoder

        if stable_count >= 10:
            left_limit = current_encoder
            break

    print(f"Left limit: {left_limit}")

    # 右極限
    print("Moving to right limit...")
    right_limit = None
    stable_count = 0
    previous_encoder = None
    while True:
        move_motor(client, unit, 0x0001, 1.0, 50)  # 嘗試大範圍移動
        current_encoder = read_encoder_raw(client, unit)
        print(f"Current encoder: {current_encoder}")

        if previous_encoder == current_encoder:
            stable_count += 1
        else:
            stable_count = 0

        previous_encoder = current_encoder

        if stable_count >= 5:
            right_limit = current_encoder
            break

    print(f"Right limit: {right_limit}")

    # 計算零點
    zero_encoder = (left_limit + right_limit) // 2
    print(f"Zero position set to: {zero_encoder}")

    print(f"Left limit: {left_limit}, Left limit angle: {(left_limit - zero_encoder) * 360.0 / resolution:.2f} degrees")
    print(f"Right limit: {right_limit}, Right limit angle: {(right_limit - zero_encoder) * 360.0 / resolution:.2f} degrees")


    # 移動到零點
    move_to_zero_position(client, unit, read_encoder_raw(client, unit), zero_encoder, 50, resolution)
    return zero_encoder, left_limit, right_limit

# 主程式入口
def main():
    client = ModbusClient(
        method='rtu',
        port='/dev/LRttyUSB',
        baudrate=9600,
        parity='N',
        stopbits=2,
        bytesize=8,
        timeout=3
    )

    UNIT = 0x00
    resolution = 3600 * 4  # 編碼器分辨率

    if not client.connect():
        print("Cannot connect to Modbus device.")
        return

    print("Connected")

    # 執行歸零校準
    zero_encoder, left_limit, right_limit = zero_calibration(client, UNIT, resolution)
    print(f"Calibration complete. Zero encoder: {zero_encoder}, Left limit: {left_limit}, Right limit: {right_limit}")
    print(f"Left limit angle: {(left_limit - zero_encoder) * 360.0 / resolution:.2f} degrees")
    print(f"Right limit angle: {(right_limit - zero_encoder) * 360.0 / resolution:.2f} degrees")


    # 進入角度控制模式
    current_angle = 0  # 當前角度（以歸零點為0度）
    while True:
        user_input = input("Enter target angle (e.g., 20 or -15), or 'q' to quit: ").strip()
        if user_input.lower() == 'q':
            print("Exiting angle control mode.")
            break

        try:
            target_angle = float(user_input)
            left_limit_angle = (left_limit - zero_encoder) * 360.0 / resolution
            right_limit_angle = (right_limit - zero_encoder) * 360.0 / resolution
            buffer_zone = 5.0  # 緩衝範圍

            # 檢查目標角度是否在合法範圍內
            if (target_angle >= left_limit_angle - buffer_zone or 
                target_angle <= right_limit_angle + buffer_zone):
                print(f"Invalid angle. Target angle must be within {right_limit_angle + buffer_zone:.2f} to {left_limit_angle - buffer_zone:.2f} degrees.")
                continue


            # 如果角度合法，進行移動
            while True:
                # 計算角度差並決定旋轉方向
                current_encoder = read_encoder_raw(client, UNIT)
                current_angle = (current_encoder - zero_encoder) * 360.0 / resolution
                angle_diff = target_angle - current_angle

                # 如果在允許的誤差範圍內，停止調整
                if abs(angle_diff) < 0.5:
                    client.write_register(0x004F, 0x0000, unit=UNIT)  # 停止馬達
                    print(f"Reached target angle: {current_angle:.2f} degrees")
                    break

                # 設置旋轉方向和目標圈數
                direction = 0x0000 if angle_diff > 0 else 0x0001
                target_circle = abs(angle_diff / 360.0)
                move_motor(client, UNIT, direction, target_circle, 100)

                print(f"Current angle: {current_angle:.2f} degrees, Target angle: {target_angle:.2f} degrees")
                time.sleep(0.1)

        except ValueError:
            print("Invalid input. Please enter a valid angle or 'q' to quit.")

        client.close()
        print("Connection closed")


if __name__ == "__main__":
    main()

