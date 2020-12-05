\page dependencies External library dependencies and build options

# List of all MRPT dependencies

## 1. Mandatory

### `eigen3`

MRPT crucially relies on Eigen3, so this dependency is mandatory. Still, for
convenience of users, if Eigen3 is not present in the system MRPT's CMake build
system will automatically switch to an embedded version of this library.

Read more about the integration of MRPT and Eigen3 in
https://www.mrpt.org/Matrices_vectors_arrays_and_Linear_Algebra_MRPT_and_Eigen_classes

## 2. Optional but strongly recommended

### `opencv`

This is used for almost everything related to computer vision in MRPT.
Even MRPT-specific computer vision functions or classes cannot work without
OpenCV since the basic structure for holding images is OpenCV's `cv::Mat`.

If not present, the following classes will raise an exception upon usage:
- mrpt::img::CImage. Note that creating mrpt::img::CImage objects in the "external storage mode" will not raise an exception unless you really access the image contents.
- Everything needing access to an image (loading, saving, processing, etc.).


### `wxwidgets`

If this library is not present in the system, the following will raise an exception upon usage:

- All window classes in mrpt::gui except those based on nanogui.
- The function  mrpt::hwdrivers::prepareVideoSourceFromUserSelection()

Also, most GUI apps will not be built if wxWidgets is not found.

### `opengl`

This optional library is required to render 3D graphics. It is fundamentally
used by the classes in mrpt::opengl.

Note however that nothing prevents you from creating objects in mrpt::opengl,
manipulating them, building 3D scenes, saving them to disk, etc. even
without OpenGL support.
An exception will be raised only if you try to **render** them in
a mrpt::gui::CDisplayWindow3D or by any other means.


## 3. Optional

### `libftdi`

This optional library is needed for interfacing FTDI USB chips in the FIFO parallel mode.

If not present, the following classes will raise exceptions upon usage:
- mrpt::comms::CInterfaceFTDI
- All its children classes.


### `liblas`

The [ASPRS LiDAR LAS](http://www.liblas.org) file format for point cloud datasets.
If available, it will provide the functionality of loading/saving
mrpt::maps::CPointsMaps to LAS files.

See: \ref mrpt_maps_liblas_grp

Since MRPT 1.5.0, building MRPT against liblas is not required. In turn, user
programs requiring this feature must make sure of adding the required compiler
and linker flags to their programs, and including the additional
file `#include <mrpt/maps/CPointsMap_liblas.h>`.

Install libLAS in Ubuntu/Debian with: `sudo apt-get install liblas-dev liblas-c-dev`


### `libpcap`

This optional library is needed to support reading/writing PCAP files (Network dump files) in these classes:

- mrpt::hwdrivers::CVelodyneScanner


### `libusb`

This optional library is needed as a dependency of the embedded version of libfreenect for communicating with Kinect sensors.

If not present, the following classes will raise exceptions upon usage:
- mrpt::hwdrivers::CKinect ( unless a different underlying implementation is used, see https://www.mrpt.org/Kinect_and_MRPT )


### `ZeroMQ`

ZeroMQ is not required to compile MRPT, but if the user application uses it,
then MRPT offers a few functions for sending and receiving MRPT objects via
ZMQ sockets.

See:
- Available functions in: \ref noncstream_serialization_zmq
- [Example code](https://github.com/MRPT/mrpt/tree/master/doc/mrpt-zeromq-example):
	- Publisher: [main_pub.cpp](https://github.com/MRPT/mrpt/blob/master/doc/mrpt-zeromq-example/main_pub.cpp)
	- Subscriber: [main_sub.cpp](https://github.com/MRPT/mrpt/blob/master/doc/mrpt-zeromq-example/main_sub.cpp)

### `suitesparse`

This suite provides many efficient sparse algebra routines, e.g. CHOLMOD.

SuiteSparse's CSparse library: Mandatory, but an embedded version exists if the library is not found in the system.

Used in these classes:
	- mrpt::math::CSparseMatrix relies on the CSparse library.
