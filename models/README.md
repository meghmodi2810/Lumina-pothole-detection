# Models Directory

This directory stores trained YOLOv8 and YOLOv8-seg weights:
- `road_damage_seg_best.pt`: High-accuracy instance segmentation model (6.7 MB)
- `peterhdd_best.pt`: YOLOv8s bounding box detection model (22.5 MB)
- `pothole_seg_nano.pt`: Keremberke YOLOv8n segmentation model (6.7 MB)

Note: Heavy `.pt` model binaries are ignored by git to keep the repository fast and lightweight.
The application automatically retrieves or uses locally loaded weights.
