# aruco_ros2

A ros2 package for [detecting ArUco markers](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html). 
This package provides a ros2 node that listens to a camera image topic, detects all markers using OpenCV, and publishes them as an array, including their poses relative to the camera frame.
It has been tested in ros2 Humble, Iron, and Jazzy.

## Dependencies

The package depends at the very leas on ros2 and OpenCV. You can install OpenCV with:

```bash
sudo apt install libopencv-dev
```

## build

```
colcon build
```

## Run

```
source install/setup.bash
ros2 run aruco_ros2 aruco_ros2 --ros-args \
  -p marker_size:=0.1 \
  -p camera_frame:=camera_rgb_optical_frame \
  -p image_topic:=/camera/color/image_raw \
  -p camera_info_topic:=/camera/color/camera_info \
  -p dictionary:=DICT_4X4_1000
```

A `MarkerArray` message is publish to the `/aruco/markers` topic

```bash
$ ros2 topic echo /aruco/markers
header:
  stamp:
    sec: 178
    nanosec: 814000000
  frame_id: camera_rgb_optical_frame
markers:
- header:
    stamp:
      sec: 178
      nanosec: 800000000
    frame_id: camera_rgb_optical_frame
  id: 203
  pose:
    header:
      stamp:
        sec: 178
        nanosec: 800000000
      frame_id: camera_rgb_optical_frame
    pose:
      position:
        x: -0.0258284514368117
        y: 0.10862779162531593
        z: 0.8941065429802411
      orientation:
        x: 0.9996631084061267
        y: 0.00220962443742458
        z: -0.003103434873084155
        w: -0.025674032477091293
  pixel_x: 273.0
  pixel_y: 276.0
---
```

An image with the marker coord axes drawn is publish to `/aruco/result`

![Example](static/frame0000.jpg)
![Example](static/screenshot.jpg)

The available aruco dictionaries are

```
DICT_4X4_50
DICT_4X4_100
DICT_4X4_250
DICT_4X4_1000
DICT_5X5_50
DICT_5X5_100
DICT_5X5_250
DICT_5X5_1000
DICT_6X6_50
DICT_6X6_100
DICT_6X6_250
DICT_6X6_1000
DICT_7X7_50
DICT_7X7_100
DICT_7X7_250
DICT_7X7_1000
DICT_ARUCO_ORIGINAL
DICT_APRILTAG_16h5
DICT_APRILTAG_25h9
DICT_APRILTAG_36h10
DICT_APRILTAG_36h11 
```