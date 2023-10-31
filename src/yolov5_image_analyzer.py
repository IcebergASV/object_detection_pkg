import argparse
from pathlib import Path
import torch
from models.experimental import attempt_load
from utils.datasets import LoadImage
from utils.general import check_img_size, non_max_suppression, scale_coords
from utils.torch_utils import select_device

def detect_objects(image_path, weights_path, conf_thres=0.25, iou_thres=0.45, img_size=640):
    # Initialize device
    device = select_device('')
    
    # Load model
    model = attempt_load(weights_path, map_location=device)
    
    # Set image size
    img_size = check_img_size(img_size, s=model.stride.max())
    
    # Load image
    img = LoadImage(image_path, img_size=img_size)
    
    # Get detections
    pred = model(img)
    pred = non_max_suppression(pred, conf_thres, iou_thres)
    
    # Store detection results
    detections = []
    
    for det in pred[0]:
        det = det.cpu()
        det = scale_coords(img.shape[2:], det[:, :4], img.shape[:2]).round()
        det = det.numpy().astype(int)
        x1, y1, x2, y2 = det[0]
        label = int(det[5])
        conf = float(det[4])
        
        # Store the detection information
        detection_info = {
            "label": label,
            "confidence": conf,
            "bbox": {"xmin": x1, "ymin": y1, "xmax": x2, "ymax": y2}
        }
        detections.append(detection_info)
    
    return detections

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--image', type=str, help='Input image path', default='/path/to/temp_image.jpg')
    parser.add_argument('--weights', type=str, help='Model weights path', default='/path/to/best.pt')
    args = parser.parse_args()
    
    detections = detect_objects(args.image, args.weights)
    
    # Print or further process the list of detections
    for idx, detection in enumerate(detections):
        print(f'Detection {idx + 1}:')
        print(f'Label: {detection["label"]}')
        print(f'Confidence: {detection["confidence"]}')
        bbox = detection["bbox"]
        print(f'Bounding Box: (xmin={bbox["xmin"]}, ymin={bbox["ymin"]}, xmax={bbox["xmax"]}, ymax={bbox["ymax"]})')
