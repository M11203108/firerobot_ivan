from pymodbus.client.sync import ModbusSerialClient as ModbusClient

# 設定Modbus連線
client = ModbusClient(method='rtu', port='/dev/batteryttyUSB4', baudrate=9600, timeout=1, parity='N', stopbits=1, bytesize=8)

if client.connect():
    print("Connected to Modbus device")

    try:
        # 假設你要讀取的寄存器範圍（這是範例，請依實際說明書更新地址）
        # 假設從寄存器地址 0x00 (起始地址) 開始，讀取2個寄存器 (總電壓之類的數據)
        address = 0x12  # 起始寄存器地址，根據你的需求調整
        count = 2       # 需要讀取的寄存器數量，根據你的需求調整
        
        # 讀取保持寄存器
        result = client.read_holding_registers(address, count, unit=1)
        
        if result.isError():
            print("讀取錯誤: ", result)
        else:
            # 解析返回的數據
            registers = result.registers
            print("返回寄存器數據: ", registers)

            # 如果讀取的數據是電壓，你可能需要根據說明書中的單位進行轉換
            # 假設返回的數據是以 10mV 為單位的總電壓
            total_voltage = (registers[0] << 8) + registers[1]  # 假設是兩個寄存器組合
            total_voltage = total_voltage / 100.0  # 假設單位是10mV，除以100轉成V
            print(f"總電壓: {total_voltage} V")

    except Exception as e:
        print("讀取錯誤: ", e)

    # 斷開Modbus連線
    client.close()
else:
    print("無法連接到Modbus設備")
