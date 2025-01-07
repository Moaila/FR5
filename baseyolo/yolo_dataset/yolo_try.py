from ultralytics import YOLO
import cv2
import os
# 加载YOLOv8官方预训练模型
print("[INFO] 加载YOLOv8官方预训练模型...")
model = YOLO('yolov8n.pt')  
# 图片路径
image_path = "/home/newplace/FR5/yolo_dataset/images/train/frame_20241224_160111.jpg"  
output_path = "/home/newplace/FR5/yolo_dataset/images/val/frame_20241224_160111.jpg"
# 加载图片
print("[INFO] 读取图片...")
img = cv2.imread(image_path)
if img is None:
    print("[ERROR] 图片加载失败，请检查路径！")
    exit()
# 检测物体
print("[INFO] 开始检测...")
results = model(image_path)
# 绘制检测结果
print("[INFO] 绘制检测结果...")
annotated_img = results[0].plot()  # 将检测结果绘制到图片上
# 保存标记后的图片
print(f"[INFO] 保存标记后的图片到: {output_path}")
cv2.imwrite(output_path, annotated_img)
# 打印检测框信息
for result in results:
    print("检测到的框坐标：", result.boxes.xyxy)  # 检测框坐标
    print("检测到的置信度：", result.boxes.conf)  # 置信度
    print("检测到的类别ID：", result.boxes.cls)   # 类别ID

print("[INFO] 处理完成，标记图片已保存。")