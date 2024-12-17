import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
import glob



for i in range(0, 21):
    img_path = f"./shaoyigelei/{i}.png"
    # 检查图片是否存在
    if not os.path.exists(img_path):
        continue

    plt.figure(figsize=(15, 10))
    img = cv2.imread(img_path)
    plt.imshow(img[:, :, ::-1])
    plt.axis('off')

    yolo_txt_path = img_path.replace('png', 'txt')

    try:
        with open(yolo_txt_path, 'r') as f:
            lines = f.readlines()

        lines = [x.strip() for x in lines]

        label = np.array([x.split() for x in lines], dtype=np.float32)

        # 物体类别
        class_list = ["luoding", "chilun", "luomu","luoshuang"]


        # 类别的颜色
        class_color = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0),(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0),(255, 0, 0), (0, 255, 0)]
        # 关键点的顺序
        keypoint_list = ["0", "1"]
        # 关键点的颜色
        keypoint_color = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0),(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0),(255, 0, 0), (0, 255, 0)]

        # 绘制检测框
        img_copy = img.copy()
        h, w = img_copy.shape[:2]
        for id, l in enumerate(label):
            # label_id ,center x,y and width, height
            label_id, cx, cy, bw, bh = l[0:5]
            label_text = class_list[int(label_id)]
            # rescale to image size
            cx *= w
            cy *= h
            bw *= w
            bh *= h

            # draw the bounding box
            xmin = int(cx - bw / 2)
            ymin = int(cy - bh / 2)
            xmax = int(cx + bw / 2)
            ymax = int(cy + bh / 2)
            cv2.rectangle(img_copy, (xmin, ymin), (xmax, ymax), class_color[int(label_id)], 2)
            cv2.putText(img_copy, label_text, (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, class_color[int(label_id)], 1)

        # display the image
        plt.figure(figsize=(15, 10))
        plt.imshow(img_copy[:, :, ::-1])
        plt.axis('off')

        img_copy = img.copy()
        h, w = img_copy.shape[:2]
        for id, l in enumerate(label):
            # label_id ,center x,y and width, height
            label_id, cx, cy, bw, bh = l[0:5]
            label_text = class_list[int(label_id)]
            # rescale to image size
            cx *= w
            cy *= h
            bw *= w
            bh *= h

            # draw the bounding box
            xmin = int(cx - bw / 2)
            ymin = int(cy - bh / 2)
            xmax = int(cx + bw / 2)
            ymax = int(cy + bh / 2)
            cv2.rectangle(img_copy, (xmin, ymin), (xmax, ymax), class_color[int(label_id)], 1)
            cv2.putText(img_copy, label_text, (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, class_color[int(label_id)], 1)

            # draw 17 keypoints, px,py,pv,px,py,pv...
            for j in range(5, len(l), 3):
                px, py = l[j:j + 2]
                # rescale to image size
                px *= w
                py *= h
                # puttext the index
                index = int((j - 5) / 2)
                # draw the keypoints
                if(int(px)>0):
                 cv2.circle(img_copy, (int(px), int(py)), 3, (0,255,255), -1)

        plt.figure(figsize=(15, 10))
        plt.imshow(img_copy[:, :, ::-1])
        plt.axis('off')
        cv2.imwrite(f"D:\\Project\\Dian\\yolopose\\tmp\\{i}.png", img_copy)

        plt.imshow(img_copy[:, :, ::-1])
        plt.axis('off')
        plt.close()  # 关闭图像以避免内存消耗
    except ValueError as e:
        print(f"Error processing file {yolo_txt_path}: {str(e)}")
        plt.close()  # 确保即使在发生错误时也关闭图像