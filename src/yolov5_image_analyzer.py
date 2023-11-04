#!/usr/bin/env python3

import cv2
import random
import numpy as np
import time
import argparse
import os
import torch
from yolov5.utils.torch_utils import select_device
from yolov5.models.experimental import attempt_load
from yolov5.utils.general import non_max_suppression, scale_segments, xyxy2xywh
from yolov5.utils.augmentations import letterbox


#Function that plots one box in a detected object
def plot_one_box(xyxy, img, color=None, label=None, line_thickness=None):
    # Plots one bounding box on image img
    tl = line_thickness or round(0.002 * max(img.shape[0:2])) + 1  # line thickness
    color = color or [random.randint(0, 255) for _ in range(3)]
    c1, c2 = (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3]))
    cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
    if label:
        tf = max(tl - 1, 1)  # font thickness
        t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
        c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
        cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # filled
        cv2.putText(img, label, (c1[0], c1[1] - 2), 0, tl / 3, [225, 255, 255], thickness=tf, lineType=cv2.LINE_AA)


# Function to detect objects in an image
def detect_objects(image_path, weights_path, output_image_path):
    device = select_device('')
    model = attempt_load(weights_path, device=device)

    # Define random colors for each class
    random.seed(0)
    colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(model.names))]

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
                label = f'{model.names[int(cls)]} {conf:.2f}'
                plot_one_box(xyxy, img0, label=label, color=colors[int(cls)], line_thickness=3)

    # Save the output image with detected objects
    cv2.imwrite(output_image_path, img0)

def main():
    # Specify the input image path and model weights path
    input_image_path = '/home/david/Documents/temp_image.jpg'
    weights_path = '/home/david/Documents/ObjectDetection/buoy_detector_yolov_5_pytorch.pt'
    output_image_path = '/home/david/Documents/output_image.jpg'

    # Detect objects in the input image
    detect_objects(input_image_path, weights_path, output_image_path)

    # Clean up

if __name__ == '__main__':
    main()
