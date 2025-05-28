from pymodbus.client.sync import ModbusSerialClient as ModbusClient
import time
import struct

def float_to_hex(f):
    packed = struct.pack('!f', f)
    hex_str = packed.hex()
    return hex_str

def registers_to_float(high, low):
    packed = struct.pack('!HH', high, low)
    return struct.unpack('!f', packed)[0]

client = ModbusClient(
    method='rtu',
    port='/dev/updottyUSB',
    # port='/dev/LRttyUSB',
    baudrate=9600,
    parity='N',
    stopbits=2,
    bytesize=8,
    timeout=3
)

circle = 0.25  # 設定的目標圈數
set_rpm = 30  # 設定轉速
set_rpm_1 = 30  # 設定轉速
direction = 0x0001

ieee_754_rpm = float_to_hex(set_rpm)
ieee_754_rpm_high = int('0x'+ieee_754_rpm[0:4], 16)
ieee_754_rpm_low = int('0x'+ieee_754_rpm[4:8], 16)

ieee_754_rpm_1 = float_to_hex(set_rpm_1)
ieee_754_rpm_high_1 = int('0x'+ieee_754_rpm[0:4], 16)
ieee_754_rpm_low_1 = int('0x'+ieee_754_rpm[4:8], 16)

ieee_circle = float_to_hex(circle)
ieee_circle_high = int('0x'+ieee_circle[0:4], 16)
ieee_circle_low = int('0x'+ieee_circle[4:8], 16)

UNIT = 0x00
if client.connect():
    print("Connected")
else:
    print("Cannot connect")
    exit()

# set enable
enable_result = client.write_register(0x004F, 0x0001, unit=UNIT)  
enable_read = client.read_holding_registers(0x004F, 1, unit=UNIT)  

if enable_result.isError():
    print("Enable error")
else:
    print("Enable success")

# set rpm
client.write_register(0x0053, ieee_754_rpm_high, unit=UNIT)
client.write_register(0x0054, ieee_754_rpm_low, unit=UNIT)
print(f"Set {set_rpm} rpm")

# 設置電機轉向
client.write_register(0x0050, direction, unit=UNIT)
print("Set direction")

# 設置電機轉動的圈數
client.write_register(0x0055, ieee_circle_high, unit=UNIT)
client.write_register(0x0056, ieee_circle_low, unit=UNIT)
print("Set circle")

time.sleep(3)
client.write_register(0x0053, ieee_754_rpm_high_1, unit=UNIT)
client.write_register(0x0054, ieee_754_rpm_low_1, unit=UNIT)
print(f"Set {set_rpm_1} rpm")

ENCODER_HIGH_REG = 0x0007
ENCODER_LOW_REG = 0x0008

resolution = 3600*4  # 編碼器分辨率為3600
cmd=0.15
# 每秒讀取一次當前轉動的圈數
try:
    while True:
        # 讀取編碼器當前碼值
        position_high = client.read_holding_registers(ENCODER_HIGH_REG, 1, unit=UNIT)
        position_low = client.read_holding_registers(ENCODER_LOW_REG, 1, unit=UNIT)

        high_value = position_high.registers[0]
        low_value = position_low.registers[0]

        encoder_value = (high_value << 16) | low_value

        if encoder_value & 0x80000000:
            encoder_value = -(0xFFFFFFFF - encoder_value + 1)

        current_circle = encoder_value / resolution
        current_circle = abs(current_circle)

        print(f"Encoder current value (decimal): {encoder_value}")
        print(f"Current position (circles): {format(current_circle, '.4f')} circles")

        # 檢查當前圈數是否在允許的範圍內 (例如0.145到0.155之間)
        if cmd <= current_circle <= cmd+0.01:
            print("Target reached, stopping motor")
            enable_result = client.write_register(0x004F, 0x0000, unit=UNIT)  
            enable_read = client.read_holding_registers(0x004F, 0, unit=UNIT)  
            # break

        # time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopped by user")

client.close()
print("Close connection")
