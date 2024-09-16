import cv2

def get_camera_info(camera_index=0):
    # 打开摄像头
    cap = cv2.VideoCapture(camera_index)
    
    if not cap.isOpened():
        print("无法打开摄像头")
        return
    
    # 获取摄像头的基本信息
    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    fps = cap.get(cv2.CAP_PROP_FPS)
    fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    
    # 显示摄像头的详细信息
    print(f"分辨率: {width}x{height}")
    print(f"帧率: {fps} fps")
    print(f"编码格式: {fourcc}")

    # 捕获一帧图像并显示
    ret, frame = cap.read()
    if ret:
        cv2.imshow('Camera Frame', frame)
        cv2.waitKey(0)  # 按任意键关闭图像窗口
        cv2.destroyAllWindows()
    else:
        print("无法读取图像帧")
    
    # 释放摄像头资源
    cap.release()

if __name__ == "__main__":
    get_camera_info()
