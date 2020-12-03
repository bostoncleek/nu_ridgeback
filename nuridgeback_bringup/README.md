# Calibrate the Bumblebee Camera
Requires [FlyCapture SDK](https://www.flir.com/products/flycapture-sdk/)

```
rosrun camera_calibration cameracalibrator.py --approximate 0.1 --size 8x6 --square 0.025 right:=/camera/right/image_raw left:=/camera/left/image_raw right_camera:=/camera/right left_camera:=/camera/left
```

The calibration file from the `camera_calibration` package is located in `$HOME/.ros/camera_info`.

The camera's serial number is `19363403`. To retrieve camera information run:
```
rosrun pointgrey_camera_driver list_cameras
```
