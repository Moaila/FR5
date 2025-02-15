"""
@作者：李文皓
@功能：读取相机设备
@注意事项：可能会有黄色的各种警告和报错，直接运行再说，能跑就是好代码
"""
import sys
import os
import time

# 添加动态链接库路径
pyorbbecsdk_path = "/home/newplace/FR5/pyorbbecsdk"
if pyorbbecsdk_path not in sys.path:
    sys.path.append(pyorbbecsdk_path)

# 设置LD_LIBRARY_PATH
os.environ["LD_LIBRARY_PATH"] = pyorbbecsdk_path + ":" + os.environ.get("LD_LIBRARY_PATH", "")

# 导入pyorbbecsdk
try:
    from pyorbbecsdk import *
    print("pyorbbecsdk 导入成功！")
except ImportError as e:
    print(f"导入失败: {e}")
    sys.exit(1)

# 设置日志等级为ERROR
ctx = Context()
ctx.set_logger_level(OBLogLevel.ERROR)

# 查询设备列表
device_list = ctx.query_devices()
device_num = device_list.get_count()

if device_num == 0:
    print("[ERROR] 没有设备连接")
else:
    print(f"检测到 {device_num} 个设备")

# 获取设备序列号
serial_num = device_list.get_device_serial_number_by_index(0)
print(f"设备序列号为: {serial_num}")