from pymodbus.client.sync import ModbusSerialClient as ModbusClient
from pymodbus.exceptions import ModbusIOException

import numpy as np
import math
import struct
import time

client = ModbusClient(
    method='rtu',       # 通訊模式
    port='/dev/IOttyUSB', # 串口設備
    baudrate=9600,     # 波特率
    stopbits=2,          # 停止位
    bytesize=8,          # 資料位
    timeout=3            # 超時時間
)

UNIT = 0x01
coil_safe_front = 0x0002 #前安全感測器
coil_safe_back = 0x0000 #後安全感測器
coil_liquid = 0x0004 #水位感測器

IOinput_coil = coil_safe_back

if not client.connect():
    print("Cannot connect")
    exit()

try:
    while True:
        # 讀取輸入信號
        result = client.read_discrete_inputs(IOinput_coil, count=1, unit=UNIT)
        if not result.isError():
            sensor_status = result.bits[0]
            if sensor_status:
                print("感測器無訊號")
            else:
                print("感測器接收到訊號！")
        else:
            print("读取输入信号时出错")

        # 每隔一秒读取一次
        time.sleep(1)


except KeyboardInterrupt:
    print("Stop")

finally:
    client.close()
    exit()  # 結束程式





 