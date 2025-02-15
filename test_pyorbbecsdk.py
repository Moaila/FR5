"""
测试pyorbbecsdk库能否正常导入
"""
# import platform

# cpu_architecture = platform.machine()
# print("CPU架构:", cpu_architecture)

# # 这里需要强调下, x64跟AMD64本质上就是同一种架构
# # 只是说，在Ubuntu下获取的架构名称为x86_64, 在Windows下获取的架构名称为AMD64
# if cpu_architecture not in  ["x86_64", "AMD64"]:
#     print("特殊架构，不能直接使用现有动态链接库。需要自己从源码编译pyorbbecsdk")
#     print("具体步骤, 需要看我们官网(deepsenserobot.com)中pyorbbeck开发环境配置相关章节")
import os
import sys

# 查询版本信息
print("当前的Python版本为: ")
print(sys.version)
version_info = sys.version.split(" ")[0].split(".")

# 检查内置的动态链接库有没有满足版本要求的
# 根据操作系统类型, 导入不同的pyorbbecsdk动态链接库
pyorbbecsdk_path = None
if os.name == 'nt' and version_info[1] in ["10"]:
    # Windows操作系统
    pyorbbecsdk_path = os.path.join('lib', 'pyorbbecsdk', 'windows')
elif os.name == 'posix'  and version_info[1] in ["10"]:
    # Ubuntu操作系统(Linux)
    pyorbbecsdk_path = os.path.join('lib', 'pyorbbecsdk', 'linux')

if pyorbbecsdk_path is not None:
    print("\npyorbbecsdk动态链接库存放路径为: ")
    print(pyorbbecsdk_path)
    print("\n添加到python path里面")
    sys.path.append(pyorbbecsdk_path)
else:
    print("动态链接库跟Python版本不匹配。需要自己从源码编译pyorbbecsdk")
    print("具体步骤, 需要看我们官网(deepsenserobot.com)中pyorbbeck开发环境配置相关章节")

# 奥比中光 SDK
from pyorbbecsdk import *
