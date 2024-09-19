import sys
import os
import numpy as np
import cv2
import open3d as o3d
from matplotlib import pyplot as plt
import yaml
import logging

# 导入阿凯写的Orbbec工具库, 确保orbbec_utils.py跟你目前所执行的脚本在同一级目录下
from orbbecsdk_utils import *
# 将pyorbbecsdk的动态链接库所在的文件夹，添加到Python Path
add_path_pyorbbecsdk()
# 导入pyorbbecsdk
from pyorbbecsdk import *

logging.basicConfig(level=logging.INFO)

# 连接设备
ret, device = connect_device()
print(f"[INFO] 设备是否连接成功: {ret}")
