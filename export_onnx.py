"""
Lumina ONNX Model Export & Quantization Pipeline
Implements PRD Section 5.2:
- Exports PyTorch YOLOv8 models to ONNX format.
- Graph optimization and simplification for ultra-fast edge/cloud CPU inference.
"""

import os
import argparse
from ultralytics import YOLO

def export_model(model_path, imgsz=320, opset=12, simplify=True):
    print("=" * 60)
    print(" 🚀 Lumina ONNX Exporter")
    print(f" 📦 Source Model : {model_path}")
    print(f" 📐 Image Size   : {imgsz}x{imgsz}")
    print(f" ⚙️ Opset        : {opset} | Simplify: {simplify}")
    print("=" * 60)

    if not os.path.exists(model_path):
        raise FileNotFoundError(f"Model not found at: {model_path}")

    model = YOLO(model_path)
    onnx_path = model.export(
        format="onnx",
        imgsz=[imgsz, imgsz],
        dynamic=False,
        simplify=simplify,
        opset=opset
    )
    print(f"\n[Export] Model successfully converted to ONNX: {onnx_path}")
    return onnx_path

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Export YOLO model to ONNX")
    parser.add_argument("--model", type=str, default="models/road_damage_seg_best.pt", help="Path to .pt model")
    parser.add_argument("--imgsz", type=int, default=320, help="Input dimension (PRD specifies 320 for <15ms)")
    parser.add_argument("--opset", type=int, default=12, help="ONNX opset version")
    args = parser.parse_args()
    
    export_model(args.model, imgsz=args.imgsz, opset=args.opset)
