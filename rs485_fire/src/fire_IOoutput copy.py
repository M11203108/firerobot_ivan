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

if client.connect():
    print("Connected")

    coil_ball = 0x0006
    coil_motor = 0x0003
    coil_sound = 0x0002
    coil_ledL = 0x0004
    coil_ledR = 0x0005
    IOoutput_coil = coil_ledL
    IOoutput_coil2 = coil_ledR

    write_result = client.write_coil(IOoutput_coil, True, unit=UNIT)
    client.write_coil(IOoutput_coil2, True, unit=UNIT)

    if not write_result.isError():
        print(f"Write coilLED open success")
    else:
        print("Write coil open error")
        print(write_result)

    time.sleep(10)

    write_result = client.write_coil(IOoutput_coil, False, unit=UNIT)
    client.write_coil(IOoutput_coil2, False, unit=UNIT)
    
    if not write_result.isError():
        print(f"Write coilLED close success")
    else:
        print("Write coil close error")
        print(write_result)

    client.close()
    exit()

else:
    print("Cannot connect")
    exit()



 