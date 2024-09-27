import serial
import glob

def find_device_by_id(target_id):
    # 在 /dev/serial/by-id/ 下查找带有目标ID的设备
    devices = glob.glob('/dev/serial/by-id/*')
    for device in devices:
        if target_id in device:
            return device
    return None

# 设定你的目标设备ID的一部分
target_device_id = "usb-1a86_USB_Serial-if00-port0"  # 注意：这里是根据设备名匹配
target_device = find_device_by_id(target_device_id)

if target_device:
    print(f"Found device: {target_device}")
    try:
        # 配置串口参数
        ser = serial.Serial(target_device, baudrate=9600, timeout=1)

        while True:
            # 读取温度数据
            data = ser.readline().decode('utf-8').strip()
            if data:
                print(f"读取到的温度数据: {data}")

    except serial.SerialException as e:
        print(f"串口错误: {e}")

    finally:
        ser.close()
else:
    print("未找到设备")
import serial
import glob

def find_device_by_id(target_id):
    # 在 /dev/serial/by-id/ 下查找带有目标ID的设备
    devices = glob.glob('/dev/serial/by-id/*')
    for device in devices:
        if target_id in device:
            return device
    return None

# 设定你的目标设备ID的一部分
target_device_id = "1a86_7523"  # 注意：这里是根据设备名匹配
target_device = find_device_by_id(target_device_id)

if target_device:
    print(f"Found device: {target_device}")
    try:
        # 配置串口参数
        ser = serial.Serial(target_device, baudrate=9600, timeout=1)

        while True:
            # 读取温度数据
            data = ser.readline().decode('utf-8').strip()
            if data:
                print(f"读取到的温度数据: {data}")

    except serial.SerialException as e:
        print(f"串口错误: {e}")

    finally:
        ser.close()
else:
    print("未找到设备")
