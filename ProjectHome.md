**All codes in this project have been developing by _Dynamic Human-Robot Interaction (DHRI) research group_ (Prof. Dongheui Lee, http://www.lsr.ei.tum.de/professors/detail/lee) in Technical University of Munich, Germany.**

---


# Research Interests #
## Human Robot Interaction ##
  * Symbolic Communication for physical Human Robot Interaction
  * Human-Robot Collaboration
## Machine Learning for (Humanoid) Robots ##
  * Imitation learning
  * Mimesis from Partial Observations
  * Manipulation Skill Transfer from Humans to Robots
  * Association and Reshaping of Motion Primitives
  * Step-by-step Learning Approach
## Computer Vision ##
  * 3D Motion Recovery from Monocular Images
  * Daily Life Human Motion Capturing

---


# Software Contents #
## 3D point cloud data processing ##
  * Calibration of one kinect sensor (Wiki:[CalibrationOneCam](CalibrationOneCam.md))
  * Calibration of two kinect sensors (Wiki:[CalibrationTwoCam](CalibrationTwoCam.md))
  * Pointcloud data filtering from one camera (Wiki:[PointcloudFilter](PointcloudFilter.md))
    * Transformation of pointcloud according to the given frame
    * Merge two pointcloud from two cameras into one pointcloud
    * Extracting pointcloud in the given workspace
    * Downsampling
    * Plane extraction
    * Clustering
  * Examples
    * Calibration of one camera and setting workspace
## Computer vision for RGB-D camera ##
  * Multiple object tracking (Wiki:[MultipleObjectTracking](MultipleObjectTracking.md))
  * Human motion tracking
## NAO robot ##
  * NAO remote controller in ROS
  * NAO sensor receiver in ROS
  * NAO visualizer (3D cad model and sensor data) in ROS (rviz)
  * Examples
## KUKA light weight robot (LWR) ##
## Machine Learning ##
## Application or Demonstration ##
  * 2012-2013 DHRI student hands-on projects
## Data ##

---


# Operating environment #
  * Programming language  : C++
  * Operating system
    * Ubuntu 12.04 (All codes have not been tested on the other systems)
    * ROS Fuete (http://www.ros.org)
  * All programs have been designed following _Model-View-Controller_ (MVC) design pattern. In each program,
    * _View_ part (if necessary) was designed by using Qt library(http://qt-project.org),
    * _Controller_ part was designed by using ROS interface, and
    * _Model_ part was designed by using platform-independent libraries as much as possible so that it can be reusable and compatible with other system environment.
    * Library dependencies for each module are referred in the wiki page for each module.

---


# Developers #
  * **Seongyong Koo (koosyong at gmail.com, http://koosy.blogspot.com)**
    * Computer vision
    * 3D point cloud data processing
    * NAO robot
  * **Matteo Saveriano (saveriano at lsr.ei.tum.de)**
    * Machine learning
    * KUKA LWR robot
  * **Kai Hu (kai.hu at tum.de)**
    * Humanoid walking
    * Imitation learning
    * NAO robot
  * **Jungmin Yoo (yoo0911 at lsr.ei.tum.de)**
    * Machine learning
    * Humanoid walking
    * NAO robot
  * **Prof. Dongheui Lee (dhlee at tum.de)**
    * Machine learning
    * Imitation learning
    * Supervisor