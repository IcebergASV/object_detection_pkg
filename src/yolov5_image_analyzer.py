#!/usr/bin/env python3

import cv2
import random
import numpy as np
import time
import argparse
import os
import torch
import json
from yolov5.utils.torch_utils import select_device
from yolov5.models.experimental import attempt_load
from yolov5.utils.general import non_max_suppression, scale_segments, xyxy2xywh
from yolov5.utils.augmentations import letterbox

# Define a structure for detected objects
def create_detected_object(class_name, confidence, x_min, x_max, y_min, y_max):
    return {
        "class_name": class_name,
        "confidence": confidence,
        "x_min": x_min,
        "x_max": x_max,
        "y_min": y_min,
        "y_max": y_max,
    }

# Function to detect objects in an image and return them as an array
def detect_objects(image_path, weights_path):
    device = select_device('')
    model = attempt_load(weights_path, device=device)

    detected_objects = []  # Initialize an empty list to hold detected objects

    # Read the image
    img0 = cv2.imread(image_path)
    img = letterbox(img0, new_shape=640)[0]
    img = img[:, :, ::-1].transpose(2, 0, 1)
    img = np.ascontiguousarray(img)

    # Run YOLOv5 model
    img = torch.from_numpy(img).to(device)
    img = img.float() / 255.0
    img = img.unsqueeze(0)
    pred = model(img)[0]
    pred = non_max_suppression(pred, 0.4, 0.5, agnostic=False)

    # Process the YOLOv5 output
    for i, det in enumerate(pred):
        if det is not None and len(det):
            det[:, :4] = scale_segments(img.shape[2:], det[:, :4], img0.shape).round()
            for *xyxy, conf, cls in reversed(det):
                class_name = model.names[int(cls)]
                confidence = conf.item()
                x_min, y_min, x_max, y_max = map(int, xyxy)
                detected_objects.append(create_detected_object(class_name, confidence, x_min, x_max, y_min, y_max))

    return detected_objects

def main():
    # Specify the input image path and model weights path
    input_image_path = '/home/david/Documents/catkin_ws/src/object_detection_pkg/temp_files/image.png'
    weights_path = '/home/david/Documents/ObjectDetection/buoy_detector_yolov_5_pytorch.pt'
    output_json_path = '/home/david/Documents/catkin_ws/src/object_detection_pkg/temp_files/detected_objects.json'

    # Detect objects in the input image and get the detected objects as an array
    detected_objects = detect_objects(input_image_path, weights_path)

    # Serialize the detected_objects list to a JSON string
    detected_objects_json = json.dumps(detected_objects)

    # Save the JSON data to a file
    with open(output_json_path, 'w') as json_file:
        json_file.write(detected_objects_json)

    # Print the file path to the standard output
    print(output_json_path)
if __name__ == '__main__':
    main()
