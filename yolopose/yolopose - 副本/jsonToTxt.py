# 将labelme标注的json文件转为yolo格式
import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
import glob
import json
import tqdm

# 物体类别
class_list = ["luoding", "chilun", "luomu", "luoshuang"]
# 关键点的顺序
keypoint_list = ["0", "1"]


def json_to_yolo(img_data, json_data):
    h, w = img_data.shape[:2]
    rectangles = []

    # 先读取所有矩形并初始化每个矩形的关键点列表
    for shape in json_data["shapes"]:
        label = shape["label"]
        points = shape["points"]
        shape_type = shape["shape_type"]

        if shape_type == "rectangle" and label in class_list:
            # Rectangle [x1, y1, x2, y2]
            rect = {
                "label": label,
                "rect": points[0] + points[1],  # [x1, y1, x2, y2]
                "keypoints_list": []
            }
            rectangles.append(rect)

    # 遍历所有形状，寻找关键点并归类到矩形内
    for shape in json_data["shapes"]:
        label = shape["label"]
        points = shape["points"]
        shape_type = shape["shape_type"]

        if label in keypoint_list and shape_type == "point":
            x, y = points[0]
            # 判断点是否在任何矩形内
            for rect in rectangles:
                x1, y1, x2, y2 = rect["rect"]
                if x1 <= x <= x2 and y1 <= y <= y2:
                    rect["keypoints_list"].append(points[0])
                    break  # 添加点后不再查找其他矩形

    # 转为yolo格式
    yolo_list = []
    for rectangle in rectangles:
        result_list = []
        label_id = class_list.index(rectangle["label"])
        x1, y1, x2, y2 = rectangle["rect"]
        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2
        width = abs(x1 - x2)
        height = abs(y1 - y2)
        # normalize
        center_x /= w
        center_y /= h
        width /= w
        height /= h

        result_list = [label_id, round(center_x, 6), round(center_y, 6), round(width, 6), round(height, 6)]

        # 处理每个矩形内的关键点
        for point in rectangle["keypoints_list"]:
            x, y = point
            x /= w
            y /= h
            result_list.extend([round(x, 6), round(y, 6), 2])

        # 如果只有一个关键点，补充一个 (0,0,0)
        if len(rectangle["keypoints_list"]) == 1:
            result_list.extend([0, 0, 0])
        # 如果有三个关键点，打印警告
        if len(rectangle["keypoints_list"]) == 3:
            print("Warning: 3 keypoints detected in one rectangle")

        yolo_list.append(result_list)

    return yolo_list


# 获取所有的图片
img_list = glob.glob("shaoyigelei/*.png")
for img_path in tqdm.tqdm(img_list):
    img = cv2.imread(img_path)
    print(img_path)
    json_file = img_path.replace('png', 'json')
    with open(json_file) as json_file:
        json_data = json.load(json_file)

    yolo_list = json_to_yolo(img, json_data)
    yolo_txt_path = img_path.replace('png', 'txt')

    with open(yolo_txt_path, "w") as f:
        for yolo in yolo_list:
            for i in range(len(yolo)):
                if i == 0:
                    f.write(str(yolo[i]))
                else:
                    f.write(" " + str(yolo[i]))
            f.write("\n")
