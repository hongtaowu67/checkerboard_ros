# checkerboard_ros
ROS package for capturing the pose of the checkerboard

Author: Hongtao Wu

Date: June 03, 2020

Part of the code is borrowed from [twu_checkerboard](https://github.com/tuw-robotics/tuw_marker_detection). This repository includes software developed by the TU-Wien.


This code is for captureing the pose of the checkerboard. It is built primary for hand-eye calibration purposes.

## Usage
Here we demonstrate the usage of the code with a depth camera (PrimeSense Carmine 1.09).

The checkerboard we used can be found in `doc/`. It is a 6x7 checkerboard. The edge length of the squre is 2.56cm if printed with a letter size paper. The checkerbaord parameters can be set in `include/capture_checkerboard.h`.

First, calibrate the intrinsic of the camera rgb and depth sensor with 
Roslaunch the camera with [MonocularCalibration](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration) and OpenNI [IntrinsicCalibration](http://wiki.ros.org/openni_launch/Tutorials/IntrinsicCalibration). This would provide you with rectified images.

Then, roslaunch the camera with
```shell
rosulaunch openni2_launch openni2.launch depth_registration:=true
```
`depth_registration:=true` gives you regirstered depth and rgb images.

Find the rectified rgb image and image info rostopic by
```shell
rostopic list
```
Set the the rgb image and image info rostopic in the `main()` of `src/capture_checkerboard.cpp`.

Then rosrun the capture node with
```shell
rosrun checkerboard_ros capture_checkerboard
```
The pose of the checkerboard will be published on the rostopic `/capture/pose_check`; the result of the pose estimation (in rgb image) will be published on the rostopic `/capture/pose_result`. Use the following to see the pose estimation result
```shell
rosrun image_view image_view image:=/capture/pose_result
```

## TODO
Write the launch file to include the camera topic and the checkerboard dimension.



