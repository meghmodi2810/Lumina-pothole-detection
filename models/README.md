# Models Directory

This directory houses the perception weights for Project Lumina:

| File | Type | Size | Status | Description |
| :--- | :--- | :--- | :--- | :--- |
| **`road_damage_seg_best.pt`** | YOLOv8n-Seg | 6.8 MB | **ACTIVE** | High-precision instance segmentation model trained on asphalt craters, potholes, and road defects. Provides exact polygon contours. |

### Note on Large Files & Git
Binary `.pt` and `.onnx` checkpoint files are excluded from Git commits via `.gitignore` to keep repository clones fast and lightweight.
For deployment to AWS EC2 or a Raspberry Pi, transfer `road_damage_seg_best.pt` directly via SCP or AWS S3.
