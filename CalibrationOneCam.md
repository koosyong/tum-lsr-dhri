# Contributor #
  * Seongyong Koo

# Introduction #
Calibration of one RGB-D camera by transforming the pointcloud data to the origin of a check board, and publish the transformed and registered RGB-D data as a ROS topic.
If you do not prepare a check board, the transformation matrix between the origin and camera can be given manually.

  * Input : subscribe topic name, publish topic name, information of check board, transformation matrix (if there is no check board)
  * Output : Publish pointcloud2 ROS message with the given topic name

# Source code #
  * Location : tum-lsr-dhri/pointcloud/dhri\_calibrationOneCam
  * Dependencies [HowToSetup](HowToSetup.md)
    * Ubuntu 12.04
    * ROS Fuete (http://www.ros.org/wiki/fuerte/Installation)
    * Qt 4.8.1 library
    * ROS openni\_kinect (http://www.ros.org/wiki/openni_kinect)
    * openCV


# Preliminaries #
  * Check board (not necessary if you can estimate the transformation matrix between the coordinates of camera and world frame)

# Build #
```
rosmake dhri_calibrationOneCam
```
# Execution #
  * Basic usage with GUI
> > Run roslaunch file of _calibrationOneCam.launch_ in the _dhri\_calibrationOneCam_ directory as follows.
```
roslaunch dhri_calibrationOneCam calibrationOneCam.launch
```
> > This launch file runs _device.launch_ and _processing.launch_ in _openni\_launch_ ros package, loads parameter files of camera and check board information, and runs _calibrationOneCam_ node with GUI.


> ![http://tum-lsr-dhri.googlecode.com/files/calibrationOneCam_main.png](http://tum-lsr-dhri.googlecode.com/files/calibrationOneCam_main.png)

> The initial values of GUI are loaded from _param\_camera.yaml_ file and the user can calibrate the values [automatically](CalibrationOneCam#Automatic_Calibration.md) or [manually](CalibrationOneCam#Manual_Calibration.md). The calibrated values can be saved as a default _param\_camera.yaml_ or as a different file name.

> You can use a different file name (_filename.yaml_) in _~/tum-lsr-dhri/pointcloud/dhri\_calibrationOneCam/_ which can be loaded as follows.
```
roslaunch dhri_calibrationOneCam calibrationOneCam.launch camera:=filename.yaml
```


  * Basic usage without GUI
> > If your parameters of camera are same as the values in _param\_camera.yaml_ and do not need to change any value, you can run this without GUI.
```
roslaunch dhri_calibrationOneCam calibrationOneCam.launch gui:=off
```
> > This also can be used with another parameter file _filename.yaml_
```
roslaunch dhri_calibrationOneCam calibrationOneCam.launch gui:=off camera:=filename.yaml
```

  * Visualization
> > In order to check the transformed pointcloud data based on the _origin_ coordinate with published TF message, run rviz with config file _calibrationOneCam.vcg_ in the _dhri\_calibrationOneCam_ directory as follows.
```
rosrun rviz rviz -d ~/tum-lsr-dhri/pointcloud/dhri_calibrationOneCam/calibrationOneCam.vcg
```

# Parameter of the camera geometry information #
_calibrationOneCam.launch_ automatically loads a default parameter file of the camera information, _tum-lsr-dhri/pointcloud/dhri\_calibrationOneCam/param\_camera.yaml_, which includes the 3D position(x,y and z) and orientation(yaw, pitch and roll) values of a camera as follows.
```
/dhri/calibrationOneCam/camera/x: -0.561139
/dhri/calibrationOneCam/camera/y: 0.26664
/dhri/calibrationOneCam/camera/z: 0.902885
/dhri/calibrationOneCam/camera/yaw: -99.03
/dhri/calibrationOneCam/camera/pitch: -3.31457
/dhri/calibrationOneCam/camera/roll: -149.18
```

There are three ways to change the parameter values of the camera.
  * Directly change the values in the file
  * [Automatic calibration using a check board](CalibrationOneCam#Automatic_Calibration.md)
  * [Manual calibration using rviz](CalibrationOneCam#Manual_Calibration.md)

# Automatic Calibration #

  * Parameter of the checkboard specification
_calibrationOneCam.launch_ automatically loads a parameter file of the checkboard param_```
/dhri/calibrationOneCam/checkboard/nW: 10
/dhri/calibrationOneCam/checkboard/nH: 7
/dhri/calibrationOneCam/checkboard/size: 0.07
```_

# Manual Calibration #