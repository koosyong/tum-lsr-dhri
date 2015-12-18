# Contributor #
  * Seongyong Koo

# Introduction #
Filtering point cloud data from one or two RGB-D cameras using pointcloud library (http://www.pointclouds.org).
The following functions are available selectively at the same time according to the given function list and corresponding parameters in [param.yaml](PointcloudFilter#Parameter.md).
  * Transformation of pointcloud according to the given frame
  * Merge two pointcloud from two cameras into one pointcloud
  * Extracting pointcloud in the given workspace
  * Downsampling
  * Plane extraction
  * Segmentation
The filtered point cloud data is published as a ROS message with the given topic name.

# Source code #
  * Location : tum-lsr-dhri/pointcloud/dhri\_calibrationOneCam
  * Dependencies [HowToSetup](HowToSetup.md)
    * Ubuntu 12.04
    * ROS Fuete (http://www.ros.org/wiki/fuerte/Installation)
    * Qt 4.8.1 library
    * ROS openni\_kinect (http://www.ros.org/wiki/openni_kinect)
    * pointcloud library (pcl) (http://www.pointclouds.org)


# Preliminaries #
If you want to transform the point cloud data based on the world frame, calibrate your cameras first by [CalibrationOneCam](CalibrationOneCam.md) or [CalibrationTwoCam](CalibrationTwoCam.md) to generate TF message of your camera and world coordinates.

# Build #
```
rosmake dhri_pointcloudFilter
```

# Execution #
Run roslaunch file of pointcloudFilter.launch in the dhri\_dhri\_pointcloudFilter directory as follows.
```
roslaunch dhri_pointcloudFilter pointcloudFilter.launch
```
You can use a different file name (filename.yaml) in ~/tum-lsr-dhri/pointcloud/dhri\_pointcloudFilter/ which can be loaded as follows.
```
roslaunch dhri_pointcloudFilter pointcloudFilter.launch param:=filename.yaml
```
# Parameter #
```
/dhri/pointcloudFilter/sub/num: 2
/dhri/pointcloudFilter/sub/topic1: '/camera1/depth_registered/points'
/dhri/pointcloudFilter/sub/frame1: '/origin'
/dhri/pointcloudFilter/sub/topic2: '/camera2/depth_registered/points'
/dhri/pointcloudFilter/sub/frame2: '/origin'
/dhri/pointcloudFilter/pub/topic: '/dhri/points/filtered'
/dhri/pointcloudFilter/pub/frame: '/origin'
/dhri/pointcloudFilter/workspace/on: 1
/dhri/pointcloudFilter/workspace/x: 0.
/dhri/pointcloudFilter/workspace/y: 0.
/dhri/pointcloudFilter/workspace/z: 0.
/dhri/pointcloudFilter/workspace/width: 0.7
/dhri/pointcloudFilter/workspace/height: 0.35
/dhri/pointcloudFilter/workspace/zheight: 0.35
/dhri/pointcloudFilter/workspace/topic: '/dhri/workspace'
/dhri/pointcloudFilter/downsampling/on: 1
/dhri/pointcloudFilter/downsampling/leaf: 0.01
/dhri/pointcloudFilter/planeExtraction/on: 0
/dhri/pointcloudFilter/planeExtraction/numPlane: 1
/dhri/pointcloudFilter/segmentation/on: 1
/dhri/pointcloudFilter/segmentation/tolerance: 0.02
/dhri/pointcloudFilter/segmentation/minSize: 20
/dhri/pointcloudFilter/segmentation/maxSize: 25000
/dhri/pointcloudFilter/segmentation/topicRGB: '/dhri/points/segmentedRGB'
/dhri/pointcloudFilter/segmentation/topicID: '/dhri/points/segmentedID'
```