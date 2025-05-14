from ultralytics import YOLO

# Load a model
model = YOLO("seg_model/yolo_training_preparation/runs/segment/train/weights/best.pt")  # load a custom model

# Predict with the model
results = model("seg_model/yolo_training_preparation/predict_dataset/box1.jpg")  # predict on an image

# Access the results
for result in results:
    xy = result.masks.xy  # mask in polygon format
    xyn = result.masks.xyn  # normalized
    masks = result.masks.data  # mask in matrix format (num_objects x H x W)
