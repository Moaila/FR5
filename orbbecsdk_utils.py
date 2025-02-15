'''奥比中光pyorbbecsdk工具库
封装了Orbbec SDK部分API，使用起来更简单且稳定。
------------------------------------------------
@作者: 阿凯爱玩机器人
@QQ: 244561792
@微信: xingshunkai
@邮箱: xingshunkai@qq.com
@网址: deepsenserobot.com
@B站: "阿凯爱玩机器人"
'''
import os
import sys
import time
import numpy as np
import cv2
import open3d as o3d
import logging

# 获取logger实例
logger = logging.getLogger("OrbbecSDK Utils")
# 指定日志的最低输出级别
logger.setLevel(logging.INFO)



def add_path_pyorbbecsdk():
    '''将pyorbbecsdk的动态链接库添加到Python Path中'''
    pyorbbecsdk_path = None
    if os.name == 'nt':
        # Windows操作系统
        pyorbbecsdk_path = os.path.join('lib', 'pyorbbecsdk', 'windows')
    elif os.name == 'posix':
        # Ubuntu操作系统(Linux)
        pyorbbecsdk_path = os.path.join('lib', 'pyorbbecsdk', 'linux')
    sys.path.append(pyorbbecsdk_path)

# 将pyorbbecsdk添加到动态链接库
add_path_pyorbbecsdk()
# 导入pyorbbecsdk
from pyorbbecsdk import *

def connect_device(serial_num=None):
    '''连接设备'''
    # 设置日志等级为ERROR 
    # 这样不会频繁的打印日志信息
    contex = Context()
    contex.set_logger_level(OBLogLevel.ERROR)

    # 查询设备列表 
    device_list = contex.query_devices()
    # 获取设备个数
    device_num = device_list.get_count()

    device = None
    if device_num == 0:
        logger.error("[ERROR]没有设备连接")
        return False, None
    else:
        # 没有指定序列号
        if serial_num is None:
            logger.info(f"[INFO] 检测到{device_num}个设备")
            # 获取特定索引下的设备序列号
            serial_num = device_list.get_device_serial_number_by_index(0)
            logger.info(f"[INFO]设备序列号为: {serial_num}")
        try:
            if "device" in globals():
                logger.info("[INFO]设备已经创建过, 在创建前先释放设备。")
                # 断开设备, 重新创建连接
                try:
                    # 设备断开连接
                    device.reboot()
                    # 删除设备
                    del device
                except Exception as e:
                    logger.error("[ERROR]在断开设备的时候，出现报错 ")
                    print(e)
                    # 删除设备
                    del device
            logger.info("[INFO]重新刷新设备列表")
            # 预留3s, 给设备重新连接预留时间
            for i in range(30):
                # 重新查询设备列表 
                device_list = contex.query_devices()
                # 检查是否有设备接入
                if device_list.get_count() != 0:
                    # 检测到设备接入, 就退出循环
                    break
                time.sleep(0.1)
            
            if device_list.get_count() != 0:
                # 根据设备序列号创建设备
                device = device_list.get_device_by_serial_number(serial_num)
                logger.info("[INFO]设备成功创建连接")
                return True, device
            else:
                logger.error("[ERROR] 没有检测到设备连接")
                return False, None
        except OBError as e:
            logger.error("[ERROR] 设备连接失败, 检查是不是有其他脚本/上位机软件占用了相机设备")
            logger.error("需要将其他脚本/上位机都关掉之后， 重新当前脚本并重试")
            logger.error("当然也有可能是在当前的脚本中，相机设备已经创建了连接。 因此在重新连接前，先释放设备。")
            logger.error("详细信息: ")
            logger.error(e)
            return False, None

def init_pipeline(device):
    '''初始化管道'''
    # 创建Pipeline
    # if "pipline" in globals():
    #     # 在修改管道配置之前，需要先停止数据流传输
    #     pipeline.stop()
    #     # 删除pipeline 
    #     del pipeline
    # 将device传入Pipeline
    pipeline = Pipeline(device)
    # 创建配置信息对象
    config = Config()

    # 获取彩图选项列表
    color_profile_list = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)   
    # 获取深度图选项列表
    depth_profile_list = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)   

    # 手动创建彩色视频流配置信息
    width = 1280 # 图像宽度
    height = 720 # 图像高度
    fmt = OBFormat.MJPG # 图像格式
    fps = 30 # 帧率
    color_profile = color_profile_list.get_video_stream_profile(width, height, fmt, fps)
    # 在配置信息里面定义彩色视频流的基本信息
    config.enable_stream(color_profile)
    
    # 手动创建深度图视频流配置信息
    width = 1280 # 图像宽度
    height = 720 # 图像高度
    fmt = OBFormat.Y16 # 图像格式
    fps = 30 # 帧率
    depth_profile = depth_profile_list.get_video_stream_profile(width, height, fmt, fps)
    # 在配置信息里面定义深度图视频流的基本信息
    config.enable_stream(depth_profile)

    # 注: Gemini335不支持硬件对齐 
    # 选择软件对齐
    config.set_align_mode(OBAlignMode.SW_MODE)
    # 帧同步 ？ 
    # pipeline.enable_frame_sync()
    # 禁用LDP
    device.set_bool_property(OBPropertyID.OB_PROP_LDP_BOOL, False)
    # 开启并配置管道
    pipeline.start(config)
    return pipeline

def print_video_profile(profile):
    '''打印视频流的基本信息'''
    # 打印视频流信息
    profile_type = profile.get_type()
    print(f"视频流类型: {profile_type}")
    fmt = profile.get_format()
    print(f"视频流格式: {fmt}")
    width = profile.get_width()
    height = profile.get_height()
    fps =profile.get_fps()
    print(f"分辨率 {width} x {height} 帧率 {fps}")


def get_rgb_camera_intrisic(pipeline):
    '''获取彩色相机的内参矩阵'''
    # 获取pipeline的相机参数
    camera_param = pipeline.get_camera_param()

    # 获取彩色相机内参
    fx = camera_param.rgb_intrinsic.fx
    fy = camera_param.rgb_intrinsic.fy
    cx = camera_param.rgb_intrinsic.cx
    cy = camera_param.rgb_intrinsic.cy
    # 构造内参矩阵
    intrinsic = np.float32([
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1]])
    logger.info("[INFO] 彩色相机的内参矩阵:")
    logger.info(intrinsic)
    return intrinsic

def get_rgb_camera_intrisic_o3d(pipeline):
    '''获取Open3D格式的RGB相机内参'''
    # 获取pipeline的相机参数
    camera_param = pipeline.get_camera_param()

    # 获取彩色相机内参
    fx = camera_param.rgb_intrinsic.fx
    fy = camera_param.rgb_intrinsic.fy
    cx = camera_param.rgb_intrinsic.cx
    cy = camera_param.rgb_intrinsic.cy
    # 彩图尺寸
    img_width = camera_param.rgb_intrinsic.width
    img_height = camera_param.rgb_intrinsic.height
    # 生成Open3D的内参格式
    pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
                img_width, img_height, \
                fx, fy, cx, cy)
    return pinhole_camera_intrinsic

def color_frame_to_bgr_img(frame):
    '''将彩图数据帧转换为numpy格式的BGR彩图'''
    width = frame.get_width()
    height = frame.get_height()
    color_format = frame.get_format()
    data = np.asanyarray(frame.get_data())
    image = np.zeros((height, width, 3), dtype=np.uint8)
    if color_format == OBFormat.RGB:
        image = np.resize(data, (height, width, 3))
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    elif color_format == OBFormat.MJPG:
        image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    else:
        logger.error("[ERROR] 不支持彩图数据格式: {}".format(color_format))
        return None
    return image

def depth_frame_to_depth_img(frame):
    '''深度数据帧转换为深度图'''
    # 获取深度图宽度
    width = frame.get_width()
    # 获取深度图高度
    height = frame.get_height()
    # 获取深度图尺度
    scale = frame.get_depth_scale()
    # 转换为numpy的float32格式的矩阵
    depth_data = np.frombuffer(frame.get_data(), dtype=np.uint16)
    depth_data = depth_data.reshape((height, width))
    depth_data = depth_data.astype(np.float32) * scale
    return depth_data

def capture_color_img(pipeline, retry_num=10, timeout_ms=3500):
    '''拍照, 只获取彩图
    @pipeline: 数据管道
    @retry_num: 重试次数
    @timeout_ms: 超时等待时间, 单位ms
    '''
    # 彩图
    color_img = None
    # 深度图
    depth_img = None

    # 重复10次
    for i in range(retry_num):
        # 获取数据帧
        frames = pipeline.wait_for_frames(timeout_ms)

        if frames is None:
            logger.warn("[WARN] 数据帧获取失败, 请重试")
            continue
        else:
            logger.info("[INFO] 数据帧读取成功")

            # 从数据帧frames中获取彩图数据帧
            color_frame = frames.get_color_frame()
            if color_frame is None:
                logger.warn("[WARN] 彩图获取失败")
                continue
            else:
                # 转换为OpenCV格式的彩图数据格式
                color_img = color_frame_to_bgr_img(color_frame)
                if color_img is None:
                    logger.warn("[WARN] 彩图数据解析失败")
                    continue
                else:
                    logger.info("[INFO] 彩图获取成功")
                    return True, color_img
                
    return False, None

def capture(pipeline, retry_num=10, timeout_ms=3500):
    '''拍照, 同时采集彩图与深度图
    @pipeline: 数据管道
    @retry_num: 重试次数
    @timeout_ms: 超时等待时间, 单位ms
    '''
    # 彩图
    color_img = None
    # 深度图
    depth_img = None

    # 重复10次
    for i in range(retry_num):
        # 获取数据帧
        frames = pipeline.wait_for_frames(timeout_ms)

        if frames is None:
            logger.warn("[WARN] 数据帧获取失败, 请重试")
            continue
        else:
            logger.info("[INFO] 数据帧读取成功")

            # 从数据帧frames中获取彩图数据帧
            color_frame = frames.get_color_frame()
            if color_frame is None:
                logger.warn("[WARN] 彩图获取失败")
                continue
            else:
                # 转换为OpenCV格式的彩图数据格式
                color_img = color_frame_to_bgr_img(color_frame)
                if color_img is None:
                    logger.warn("[WARN] 彩图数据解析失败")
                    continue
                else:
                    logger.info("[INFO] 彩图获取成功")
            
            # 从数据帧frames中获取深度图数据帧
            depth_frame = frames.get_depth_frame()
            if depth_frame is None:
                logger.warn("[WARN] 深度图获取失败")
            else:
                # 转换为OpenCV格式的彩图数据格式
                depth_img = depth_frame_to_depth_img(depth_frame)
                if depth_img is None:
                    logger.warn("[WARN] 深度图数据解析失败")
                    continue
                else:
                    logger.info("[INFO] 深度图获取成功")
                    return True, color_img, depth_img
    return False, None, None

def pixel2point3d(px, py, depth_mm, camera_param):
    '''将像素坐标转换为相机坐标系下的三维坐标
    @px: 像素坐标 X轴坐标
    @py: 像素坐标 Y轴坐标
    @depth_mm: 深度值
    @camera_param: 相机参数
    '''
    # 获取彩色相机内参
    fx = camera_param.rgb_intrinsic.fx
    fy = camera_param.rgb_intrinsic.fy
    cx = camera_param.rgb_intrinsic.cx
    cy = camera_param.rgb_intrinsic.cy

    # 计算相机坐标系下的三维坐标
    cam_z = depth_mm
    cam_x = (px - cx) * cam_z / fx
    cam_y = (py - cy) * cam_z / fy

    return [cam_x, cam_y, cam_z]

def create_point_cloud(color_image, depth_image, camera_param, depth_scale=1000.0):
    '''创建点云(向量化操作)
    @color_image: 彩图
    @depth_image: 深度图
    @depth_scale: 深度图单位/尺度  
                  一般深度图单位是mm， 转换为m需要/1000.0
    '''
    # 获取彩色相机内参
    fx = camera_param.rgb_intrinsic.fx
    fy = camera_param.rgb_intrinsic.fy
    cx = camera_param.rgb_intrinsic.cx
    cy = camera_param.rgb_intrinsic.cy
    # 彩图尺寸
    img_width = camera_param.rgb_intrinsic.width
    img_height = camera_param.rgb_intrinsic.height

    # 缩放深度图
    if depth_scale != 1.0:
        depth_image = depth_image / depth_scale
    # 得到索引号
    valid_index = depth_image != 0
    # 得到有效点云的RGB数值
    color_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
    colors = color_rgb[valid_index].reshape((-1, 3)) / 255
    # 创建一个空的点云对象
    pcd = o3d.geometry.PointCloud()
    # 根据相机内参矩阵计算3D坐标
    py, px = np.indices((img_height, img_width))
    # 提取
    px_valid = px[valid_index]
    py_valid = py[valid_index]
    z = depth_image[valid_index]
    # 计算相机坐标系下的三维坐标
    x = (px_valid - cx) * z / fx
    y = (py_valid - cy) * z / fy
    points = np.stack([x, y, z], axis=-1)
    # 将3D坐标转换为点云对象
    points = points.reshape(-1, 3)
    # 填充PCD对象
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd


def empty_cache(pipeline, n_frame=30):
    '''清空缓冲区'''
    # pipeline 刚开始创建的时候图像不稳定
    # 需要跳过一些帧
    for i in range(n_frame):
        capture(pipeline)