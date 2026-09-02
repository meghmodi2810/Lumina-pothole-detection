"""
Lumina Autonomous Project - Local Model Fine-Tuning & Training Pipeline
Allows fine-tuning YOLOv8 / YOLOv8-seg on custom road datasets in this directory.

Usage:
    python train.py --data data.yaml --epochs 50 --imgsz 640 --model yolov8n.pt
    python train.py --mode segmentation --data data.yaml --epochs 50 --model yolov8n-seg.pt
"""

import os
import argparse
import yaml
from ultralytics import YOLO

def create_sample_data_yaml(target_path="data/data.yaml"):
    """Creates a sample YOLO dataset configuration if one doesn't exist."""
    os.makedirs(os.path.dirname(target_path), exist_ok=True)
    if not os.path.exists(target_path):
        sample_config = {
            "path": os.path.abspath("data"),
            "train": "images/train",
            "val": "images/val",
            "names": {
                0: "pothole"
            }
        }
        with open(target_path, "w") as f:
            yaml.dump(sample_config, f, default_flow_style=False)
        print(f"[Train] Created starter dataset template at: {target_path}")

def train_model(data_yaml, base_model="yolov8n.pt", epochs=30, imgsz=640, batch_size=8, task="detect", project_dir="runs/train"):
    """
    Fine-tunes a YOLO model using transfer learning with custom vehicular augmentations.
    """
    print("=" * 65)
    print(" 🚀 Lumina Pothole Perception Training Pipeline")
    print(f" 📦 Base Backbone : {base_model}")
    print(f" 📂 Dataset Config : {data_yaml}")
    print(f" 🔄 Epochs        : {epochs} | Batch: {batch_size} | ImgSz: {imgsz}")
    print(f" 🎯 Task Mode     : {task}")
    print("=" * 65)

    if not os.path.exists(data_yaml):
        create_sample_data_yaml(data_yaml)

    # Initialize model from pretrained weights for transfer learning
    model = YOLO(base_model)

    # Hyperparameters tailored for autonomous driving dashcam perspective
    results = model.train(
        data=data_yaml,
        epochs=epochs,
        imgsz=imgsz,
        batch=batch_size,
        project=project_dir,
        name="lumina_pothole_run",
        exist_ok=True,
        # Augmentations for road vibration, changing sunlight, and motion blur
        hsv_h=0.015,     # HSV-Hue augmentation
        hsv_s=0.7,       # HSV-Saturation
        hsv_v=0.4,       # HSV-Value (shadows / sunlight)
        degrees=10.0,    # Small rotation for road camber
        translate=0.1,   # Translation
        scale=0.3,       # Scale jitter (distant vs near obstacles)
        shear=2.0,       # Perspective tilt
        perspective=0.0005, # Camera perspective distortion
        flipud=0.0,      # Do NOT flip upside down (roads are on bottom!)
        fliplr=0.5,      # Left-right mirror is valid
        mosaic=1.0,      # Mosaic augmentation for small object detection
        mixup=0.15,      # Mixup for texture blending
        patience=15,     # Early stopping patience
        save=True,
        save_period=5,
        plots=True,
        verbose=True
    )

    print("\n[Train] Training complete!")
    best_weights = os.path.join(project_dir, "lumina_pothole_run", "weights", "best.pt")
    if os.path.exists(best_weights):
        # Copy to models directory
        dest = os.path.join("models", "lumina_finetuned_best.pt")
        import shutil
        shutil.copy(best_weights, dest)
        print(f"[Train] Deployed best weights to: {dest}")

    # Validate model
    print("\n[Train] Running final validation benchmark...")
    metrics = model.val()
    print(f"[Train] Validation mAP@0.5      : {metrics.box.map50:.4f}")
    print(f"[Train] Validation mAP@0.5:0.95 : {metrics.box.map:.4f}")
    return results

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Lumina Pothole Detection Fine-Tuning Pipeline")
    parser.add_argument("--data", type=str, default="data/data.yaml", help="Path to data.yaml")
    parser.add_argument("--model", type=str, default="models/road_damage_seg_best.pt", help="Pretrained model or backbone")
    parser.add_argument("--epochs", type=int, default=25, help="Number of training epochs")
    parser.add_argument("--batch", type=int, default=8, help="Batch size")
    parser.add_argument("--imgsz", type=int, default=640, help="Image resolution")
    parser.add_argument("--task", type=str, default="segment", choices=["detect", "segment"], help="Task mode")
    
    args = parser.parse_args()
    train_model(
        data_yaml=args.data,
        base_model=args.model,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch_size=args.batch,
        task=args.task
    )
