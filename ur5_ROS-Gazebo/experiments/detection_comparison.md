# Detection Method Comparison

Use the same wrist-camera image stream for every method.

YOLO:

```bash
roslaunch ur5_notebook detection.launch
rostopic echo /detection/stats
```

Traditional baselines:

```bash
roslaunch ur5_notebook traditional_detection.launch method:=digit
rostopic echo /traditional_detection/details

roslaunch ur5_notebook traditional_detection.launch method:=color
roslaunch ur5_notebook traditional_detection.launch method:=contour
roslaunch ur5_notebook traditional_detection.launch method:=template
roslaunch ur5_notebook traditional_detection.launch method:=hybrid
```

Useful topics:

- `/traditional_detection/results`: same `Detection2DArray` format as YOLO.
- `/traditional_detection/image_annotated`: annotated traditional detector image.
- `/traditional_detection/stats`: JSON line with method, elapsed time, and detected classes.
- `/traditional_detection/details`: JSON line with digit, color, bounding box, pixel
  center, estimated top-down world center, and whether color assisted a correction.

Suggested table columns:

| Method | Trial | Detected 0-9 | Correct count | Miss count | False count | Avg time ms |
| --- | --- | --- | --- | --- | --- | --- |

For accuracy, compare the detected classes against the task-required classes `0-9`.
