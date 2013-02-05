cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


#############################################
#############################################

set(INCLUDE $ENV{COTESYS_KINECT_DIR}/include)

include_directories(
    src/pctracking
)

set( sources
    src/tracking_gmm.cpp
    src/pctracking.cpp
    src/pctracking_weight.cpp
    src/pctracking/pcobject.cpp
    src/pctracking/pcobjectcontainer.cpp
    src/pctracking/track.hpp
#    ${INCLUDE}/pctracking/pctrack.cpp
    src/pctracking/trackcontainer.hpp
    src/pctracking/pctrackcontainer.cpp

    src/pctracking/imft.hpp
    src/pctracking/cluster.cpp
    src/pctracking/pcgmmreg.cpp
    src/pctracking/pcgmmreg_func.cpp
    src/pctracking/gaussian.cpp
 #   ${INCLUDE}/pctracking/gaussiantrack.cpp
    src/pctracking/gaussiantrackcontainer.cpp
#    ${INCLUDE}/ckinectfiltering.cpp
#    ${INCLUDE}/pcfilter/pcfilter.hpp
 )

set( headers
    src/pctracking.h
    src/pctracking_weight.h
    src/pctracking/pcobject.h
    src/pctracking/pcobjectcontainer.h
#    ${INCLUDE}/pctracking/pctrack.h
    src/pctracking/track.h
    src/pctracking/trackcontainer.h
    src/pctracking/pctrackcontainer.h
    src/pctracking/imft.h
    src/pctracking/cluster.hpp
    src/pctracking/pcgmmreg.h
    src/pctracking/pcgmmreg_func.h
    src/pctracking/gaussian.h
#    ${INCLUDE}/pctracking/gaussiantrack.h
    src/pctracking/gaussiantrackcontainer.h
#    ${INCLUDE}/ckinect.h
#    ${INCLUDE}/ckinectfiltering.h
#    ${INCLUDE}/pcfilter/pcfilter.h
)

#############################################
# GMMREG
include_directories(
    /usr/local/include/vxl/vcl
    /usr/local/include/vxl/core
    /usr/local/include/vxl/core/vnl/algo
    /usr/local/include/vxl/v3p/netlib
)
FIND_PACKAGE(VXL)
#IF(VXL_FOUND)
#  INCLUDE(${VXL_CMAKE_DIR}/UseVXL.cmake)
#ENDIF(VXL_FOUND)

#############################################
# Required 3rd party

#############################################
# http://qtnode.net/wiki/Qt4_with_cmake

find_package(Qt4)
set(QT_USE_QTOPENGL TRUE)
include(${QT_USE_FILE})

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/local/*.hpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})
qt4_wrap_cpp(moc_sources ${headers})

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINK src/*.*)

#############################################
#############################################

rosbuild_add_executable(tracking_gmm ${sources} ${headers})
target_link_libraries(tracking_gmm vnl_algo vnl vcl)
