if __name__ == '__main__':
    #训练代码
    from ultralytics import YOLO

    # Load a model
    model = YOLO('D:\\Project\\Dian\\yolopose\\weight\\yolov8n-pose.pt')

    # Train the model
    results = model.train(data='D:\\Project\\Dian\\yolopose\\coco8-pose.yaml', epochs=5000, imgsz=640)
