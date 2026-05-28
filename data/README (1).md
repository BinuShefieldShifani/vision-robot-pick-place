# Dataset

## Overview

This dataset was collected and annotated specifically for training a **YOLOv11 instance segmentation model** to detect geometric shapes on a conveyor surface for robotic pick & place automation.

| Property | Details |
|---|---|
| **Total images** | ~500 |
| **Classes** | circle, square, triangle |
| **Annotation tool** | CVAT (Computer Vision Annotation Tool) |
| **Export format** | YOLO TXT (segmentation) |
| **Environment** | Conveyor/table surface, controlled industrial lighting |
| **Camera** | Basler industrial camera |

---

## Class Distribution

| Class | ID | Description |
|---|---|---|
| Circle | 0 | Circular objects of varying sizes |
| Square | 1 | Square/rectangular objects |
| Triangle | 2 | Triangular objects at various orientations |

---

## Expected Directory Structure

When you have access to the dataset, organise it as follows before training:

```
data/
├── images/
│   ├── train/          # ~80% of images
│   ├── val/            # ~10% of images
│   └── test/           # ~10% of images
└── labels/
    ├── train/          # YOLO .txt files for train images
    ├── val/            # YOLO .txt files for val images
    └── test/           # YOLO .txt files for test images
```

Update the `path` field in `configs/config.py` to point to this directory.

---

## Annotation Format

Each image has a corresponding `.txt` file in YOLO format.
For segmentation tasks, each line contains:

```
class_id x1 y1 x2 y2 ... xn yn
```

Where `x`, `y` are normalised polygon coordinates (0.0 to 1.0).

For bounding box format:
```
class_id x_center y_center width height
```

---

## Trained Model

The trained `best.pt` model weights are **not included in this repository** (binary files are not suited for Git version control).

Model weights are available on request alongside the dataset.

---

## Requesting Access

The dataset and model weights are available for **research and educational purposes**.

To request access, please [open an issue](../../issues) with:
- Your name and affiliation
- Intended use (research / education / project)
- Brief description of your application

You will receive a Google Drive link to the full dataset and weights.
