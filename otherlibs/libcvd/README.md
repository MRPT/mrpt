# libCVD

Note, the master branch is now libCVD-2.0 which is in beta and requires C++14.

## Compiling and installing

libCVD currently has both an autoconf and cmake based build system. The
autoconf one works on any unix-like system, and is well testes. The CMake
one should work on any system but is a little newer so may be buggy in 
untested configurations.

To install on a UNIX system:

    ./configure && make && sudo make install

To verify that a few things work, you can optionally run

    make test

To build on UNIX with [CMake](https://cmake.org/):
    
	mkdir build
	cd build
	cmake -DCMAKE_BUILD_TYPE=Release .. 
	make 

and optionally:

    make test

To build on Windows, use [CMake](https://cmake.org/):

    mkdir build
    cd build
    cmake -DCMAKE_INSTALL_PREFIX=<directory> ..
    cmake --build . --target INSTALL --config Release

### Dependencies

There are no mandatory dependencies. For a reasonably complete installation you probably want:
* TooN (header only, install from source)
* libjpeg, libtiff, libpng, ffmpeg, X11, OpenGL. 
You might also want:
* libdc1394
* [libuvc](https://github.com/ktossell/libuvc)



On Ubuntu 16.04, run:
```
sudo apt-get install libjpeg-dev libpng-dev libtiff-dev libx11-dev libavformat-dev libavdevice-dev libavcodec-dev libavutil-dev libswresample-dev libglu-dev libdc1394-22
```

## System compatibility

You need a C++14 compiler. 

All libraries are optional but you will be missing features if the libraries
aren't present. The configure script will tell you what's present and what's
not.

### Ubuntu 16.04 / gcc 5

The default C++ compiler on Ubuntu 16.04 will not compile libCVD because of
a [bug in the standards compliance of the compiler.](https://stackoverflow.com/questions/34280729/throw-in-constexpr-function). If you want to use libcvd,
you will need to install a newer compiler. The easiest eay is to add the
[toolchain test PPA](https://launchpad.net/~ubuntu-toolchain-r/+archive/ubuntu/test):

    sudo add-apt-repository ppa:ubuntu-toolchain-r/test
    sudo apt-get update
    sudo apt-get install g++-7

Now you can build libcvd with either:

    CXX=g++-7 ./configure
	make
or

    mkdir build 
	cd build
    CXX=g++-7 cmake -DCMAKE_BUILD_TYPE=Release .. 
	make 


## Documentation

[![Documentation Status](https://codedocs.xyz/edrosten/libcvd.svg)](https://codedocs.xyz/edrosten/libcvd/)

Latest documentation here: https://codedocs.xyz/edrosten/libcvd/ or just run Doxygen.

## Status of unit tests
![Build Status](https://circleci.com/gh/edrosten/libcvd.svg?style=shield&circle-token=db58907af52b26d11f2c4f5de2ff3b1a59543ddc)


## News and main page

https://www.edwardrosten.com/cvd/
