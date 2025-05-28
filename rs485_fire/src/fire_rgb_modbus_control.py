import serial
import time

ser = serial.Serial('/dev/LEDttyUSB', 9600, timeout=1, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)

def send_command(command, description):
    ser.write(command)
    response = ser.read(8)  # 讀取回應，可以根據您的設備修改此值
    if not response:
        print(f"{description} error: No response")
    else:
        print(f"{description} success")

if ser.isOpen():
    print("Connected")
    
    # 設置 IC 數量
    IC_count = bytes([0xA5, 0x07, 0x00, 0xA8, 0x7D, 0x04, 0x00, 0x2C, 0xEE, 0x5A])
    send_command(IC_count, "IC set")
    
    # 設置 RGB 排列順序
    RGB_count = bytes([0xA5, 0x06, 0x00, 0xA8, 0x7D, 0x05, 0x00, 0xEE, 0x5A])
    send_command(RGB_count, "RGB set")
    
    # 設置燈光顏色 (純白色)
    # light_count = bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    # light_count = bytes([0xA5, 0x06, 0x00, 0xA0, 0xFF, 0xFF, 0xFF, 0xEE, 0x5A])
    # send_command(light_count, "Light set")
    # time.sleep(2)
    #關燈
    light_close = bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    send_command(light_close, "Light close")
    time.sleep(2)
    
    
    
else:
    print("Cannot connect")


ser.close()

