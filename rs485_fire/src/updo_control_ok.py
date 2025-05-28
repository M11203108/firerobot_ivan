from pymodbus.client.sync import ModbusSerialClient as ModbusClient
import time
import struct
from pynput import keyboard

def float_to_hex(f):
    packed = struct.pack('!f', f)
    hex_str = packed.hex()
    return hex_str

client = ModbusClient(
    method='rtu',
    port='/dev/updottyUSB',
    baudrate=9600,
    parity='N',
    stopbits=2,
    bytesize=8,
    timeout=3
)

set_rpm = 200  # 固定轉速
circle_step = 0.01  # 每次目標位置的步進
UNIT = 0x00
resolution = 3600 * 4  # 編碼器分辨率
target_position = 0  # 假想目標位置
current_direction = None  # 記錄當前方向

ieee_754_rpm = float_to_hex(set_rpm)
ieee_754_rpm_high = int('0x' + ieee_754_rpm[0:4], 16)
ieee_754_rpm_low = int('0x' + ieee_754_rpm[4:8], 16)

if client.connect():
    print("Connected")
else:
    print("Cannot connect")
    exit()

def read_encoder():
    """讀取馬達當前位置"""
    position_high = client.read_holding_registers(0x0007, 1, unit=UNIT).registers[0]
    position_low = client.read_holding_registers(0x0008, 1, unit=UNIT).registers[0]
    encoder_value = (position_high << 16) | position_low
    if encoder_value & 0x80000000:  # 處理負值
        encoder_value = -(0xFFFFFFFF - encoder_value + 1)
    return abs(encoder_value / resolution)

def start_motor(direction):
    """啟動馬達並保持運行"""
    global current_direction, target_position
    current_direction = direction
    client.write_register(0x004F, 0x0001, unit=UNIT)  # 啟用馬達
    client.write_register(0x0053, ieee_754_rpm_high, unit=UNIT)  # 設定轉速高位
    client.write_register(0x0054, ieee_754_rpm_low, unit=UNIT)  # 設定轉速低位
    client.write_register(0x0050, direction, unit=UNIT)  # 設置方向
    print(f"Motor started: direction={direction}")

def stop_motor():
    """停止馬達"""
    global current_direction
    current_direction = None
    client.write_register(0x004F, 0x0000, unit=UNIT)  # 停止馬達
    print("Motor stopped")

def update_target_position():
    """更新目標位置"""
    global target_position, current_direction
    if current_direction is None:
        return  # 沒有方向時不更新
    target_position += circle_step if current_direction == 0x0001 else -circle_step
    ieee_circle = float_to_hex(abs(target_position))
    ieee_circle_high = int('0x' + ieee_circle[0:4], 16)
    ieee_circle_low = int('0x' + ieee_circle[4:8], 16)
    client.write_register(0x0055, ieee_circle_high, unit=UNIT)  # 設置目標圈數高位
    client.write_register(0x0056, ieee_circle_low, unit=UNIT)  # 設置目標圈數低位
    print(f"Updated target position: {target_position:.4f}")

key_state = {'i': False, ',': False, 'k': False}  # 記錄按鍵狀態

def on_press(key):
    global key_state
    try:
        if key.char == 'i' and not key_state['i']:  # 向上
            key_state['i'] = True
            start_motor(0x0001)
        elif key.char == ',' and not key_state[',']:  # 向下
            key_state[','] = True
            start_motor(0x0000)
        elif key.char == 'k' and not key_state['k']:  # 停止
            key_state['k'] = True
            stop_motor()
    except AttributeError:
        pass

def on_release(key):
    global key_state
    try:
        if key.char in key_state:
            key_state[key.char] = False
            if key.char in ['i', ',']:
                stop_motor()
    except AttributeError:
        pass

try:
    print("Listening for key presses...")
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        while True:
            if current_direction is not None:
                update_target_position()
            time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopped by user")

finally:
    stop_motor()
    client.close()
    print("Close connection")
