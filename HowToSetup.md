# Introduction #

Add your content here.

# ROS Fuete #
http://www.ros.org/wiki/fuerte/Installation

# ROS openni\_kinect #
```
sudo apt-get install ros-fuerte-openni-*
```

# Qt Creator (for developer) #
http://qt-project.org/downloads

# git and smartgit (for developer) #
  * git
```
sudo apt-get install git
```
  * git client : smartgit (http://www.syntevo.com/smartgithg/index.html)
> JAVA environment is required

# vxl (for [MultipleObjectTracking](MultipleObjectTracking.md)) #
  * Download : http://vxl.sourceforge.net/#download
  * Install : http://vxl.sourceforge.net/releases/install-release.html
```
cmake -i $VXLSRC
make
sudo make install
```
If you face some problem while making library, Set 'ON' for building these basic libraries with cmake settings.
```
BUILD_BRL
BUILD_CONTRIB
BUILD_CONVERSIONS
BUILD_CORE_GEOMETRY
BUILD_CORE_IMAGING
BUILD_CORE_NUMERICS
BUILD_CORE_SERIALISATION
BUILD_CORE_UTILITIES
```

# lemon (for [MultipleObjectTracking](MultipleObjectTracking.md)) #
  * Download : http://lemon.cs.elte.hu/trac/wiki/Downloads
  * Install : http://lemon.cs.elte.hu/trac/wiki/InstallGuide