# 测试图片
import os

from ultralytics import YOLO
import cv2
import numpy as np
import sys
import json
import numpy as np

def point_in_box(point, box):
    """检查点是否在矩形框内"""
    px, py = point
    l, t, r, b = box
    return l <= px <= r and t <= py <= b

def convert_results_to_json(boxes, keypoints, class_names):
    result = {
        "version": "5.4.1",
        "flags": {},
        "shapes": []
    }
    group_id = 1

    # 创建边界框和关键点的映射
    box_groups = {}

    # 处理边界框
    for idx, box in enumerate(boxes.data):
        l, t, r, b = box[:4].astype(np.int32)
        conf, id = box[4:]
        id = int(id)
        box_label = class_names[id]

        # 保存边界框信息和分配的group_id
        box_groups[idx] = {
            "group_id": group_id,
            "box": [l, t, r, b],
            "label": box_label
        }

        result['shapes'].append({
            "label": box_label,
            "points": [[float(l), float(t)], [float(r), float(b)]],
            "group_id": group_id,
            "description": "",
            "shape_type": "rectangle",
            "flags": {},
            "mask": None
        })

        group_id += 1

    # 处理关键点
    for keypoint in keypoints.data:
        for i, (x, y, _) in enumerate(keypoint):
            point = (x, y)
            placed = False



            result['shapes'].append({
                "label": f"{i}",  # 使用关键点标签
                "points": [[float(x), float(y)]],
                "group_id": -1,
                "description": "",
                "shape_type": "point",
                "flags": {},
                "mask": None
            })

    return json.dumps(result['shapes'], indent=1)


def update_json_shapes(json_path, shapes_json):
    # 读取对应的JSON文件
    with open(json_path, 'r+') as json_file:
        data = json.load(json_file)
        data['shapes'] = json.loads(shapes_json)  # 替换shapes字段
        json_file.seek(0)  # 重置文件指针
        json_file.write(json.dumps(data, indent=4))
        json_file.truncate()  # 删除文件中剩余的部分

# 读取命令行参数
weight_path = './pose6.pt'

# 加载模型
model = YOLO(weight_path)

# 获取类别
objs_labels = model.names  # get class labels
print(objs_labels)

# 类别的颜色
class_color = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0),(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0),(255, 0, 0), (0, 255, 0)]
# 关键点的顺序
class_list = ["luoding", "chilun", "luomu", "luoshuang"]

# 关键点的颜色
keypoint_color = [(255, 0, 0), (0, 255, 0),(255, 0, 0), (0, 255, 0),(255, 0, 0), (0, 255, 0),(255, 0, 0), (0, 255, 0),(255, 0, 0), (0, 255, 0)]

media_path = "./shaoyigelei"

# 从8-9
for i in range(0, 21):
    # 读取图片
    image_name = f"{i}.png"
    image_path = os.path.join(media_path, image_name)
    json_path = os.path.join(media_path, f"{i}.json")
    frame = cv2.imread(image_path)
    frame = cv2.resize(frame, (frame.shape[1], frame.shape[0]))
    # rotate
    # 检测
    result = list(model(frame, conf=0.3, stream=True))[0]  # inference，如果stream=False，返回的是一个列表，如果stream=True，返回的是一个生成器
    boxes = result.boxes  # Boxes object for bbox outputs
    boxes = boxes.cpu().numpy()  # convert to numpy array

    # 遍历每个框
    for box in boxes.data:
        l, t, r, b = box[:4].astype(np.int32)  # left, top, right, bottom
        conf, id = box[4:]  # confidence, class
        id = int(id)
        # 绘制框
        cv2.rectangle(frame, (l, t), (r, b), (0, 0, 255), 2)
        # 绘制类别+置信度（格式：98.1%）
        cv2.putText(frame, f"{objs_labels[id]} {conf * 100:.1f}", (l, t - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 0, 255), 1)

    # 遍历keypoints
    keypoints = result.keypoints  # Keypoints object for pose outputs
    keypoints = keypoints.cpu().numpy()  # convert to numpy array

    # draw keypoints, set first keypoint is red, second is blue
    for keypoint in keypoints.data:
        for i in range(len(keypoint)):
            x, y ,_ = keypoint[i]
            x, y = int(x), int(y)
            cv2.circle(frame, (x, y), 3, (0, 255, 0), -1)
            #cv2.putText(frame, f"{keypoint_list[i]}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, keypoint_color[i], 2)

    # 使用函数
    result_json = convert_results_to_json(boxes, keypoints, objs_labels)
    update_json_shapes(json_path, result_json)

