# Contributor #
  * Seongyong Koo (koosyong at gmail.com)

# Abstract #

This paper proposes a novel model-free approach for tracking multiple moving objects from RGB-D point set data. In order to represent arbitrary objects, a flexible object model is represented by a Gaussian Mixture Model (GMM) with a tempo-spatial topological graph (TSTG). The proposed tracking framework incrementally
updates each object model and robustly tracks each point involved in each object against various changing situations in dynamic movements of multiple objects. In order to find the optimal parameters of the method to achieve both robustness and computational efficiency, the trade-off relationship between the accuracy and efficiency was examined by various experiments, and the method achieved over 97% tracking accuracy while processing 5 frames per second for the given tasks. The limitations of the method were also empirically investigated in terms of the size of points and the movement speed of the objects.
![http://tum-lsr-dhri.googlecode.com/files/architecture.png](http://tum-lsr-dhri.googlecode.com/files/architecture.png)

# Result #
[MultipleObjectTrackingResult](MultipleObjectTrackingResult.md)

# Publications #
  * Seongyong Koo, Dongheui Lee, Dong-Soo Kwon, “GMM-based 3D Object Representation and Robust Tracking in Unconstructed Dynamic Environments”, 2013 IEEE International Conference on Robotics and Automation (ICRA 2013) (accepted)
  * Seongyong Koo, Dongheui Lee, Dong-Soo Kwon, "Incremental object modeling and robust tracking of multiple objects from RGB-D point set data", Journal of Visual Communication and Image Representation (JVCI) (submitted)

# Source code #
  * Location : tum-lsr-dhri/pointcloud/dhri\_multipleObjectTracking
  * Dependencies [HowToSetup](HowToSetup.md)
    * Ubuntu 12.04
    * ROS Fuete (http://www.ros.org/wiki/fuerte/Installation)
    * Qt 4.8.1 library
    * ROS openni\_kinect (http://www.ros.org/wiki/openni_kinect)
    * openCV
    * vxl library (http://vxl.sourceforge.net/)
    * lemon library (http://lemon.cs.elte.hu/trac)


---

# How to setup #
Add your content here.  Format your content with:
  * Text in **bold** or _italic_
  * Headings, paragraphs, and lists
  * Automatic links to other wiki pages