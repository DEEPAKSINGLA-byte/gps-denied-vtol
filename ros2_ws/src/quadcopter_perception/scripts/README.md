# Segmentation demo

`experimentation.py` runs plain YOLOv8 segmentation on your webcam or an image. It needs no
drone and no ROS. (The drone uses YOLO + SAM through the `segmentation_node`; see
[docs/REFERENCE.md](../../../../docs/REFERENCE.md#segmentation-yolo--sam).)

```bash
pip install ultralytics opencv-python
python3 experimentation.py              # webcam; press q to quit
python3 experimentation.py photo.jpg    # one image
```

It shows two windows: the image with the segmentation drawn on it, and only the detected
objects (background removed). `yolov8n-seg.pt` downloads on first run.

[Demo video](https://youtu.be/XxDwwBQxVR0)
