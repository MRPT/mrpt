# How to cross compile MRPT


## Install C & C++ compilers for your target CPU

For example:

        sudo apt install gcc-8-arm-linux-gnueabihf g++-8-arm-linux-gnueabihf

## Create cmake toolchain file

        # Save as doc/toolchain-gcc8-arm.cmake
        set(CMAKE_SYSTEM_NAME Linux)
        set(CMAKE_SYSTEM_PROCESSOR arm)

        set(triple arm-linux-gnueabihf)

        set(CMAKE_C_COMPILER ${triple}-gcc-8)
        set(CMAKE_CXX_COMPILER ${triple}-g++-8)

        # Your target system image:
        set(CMAKE_FIND_ROOT_PATH /mnt/root)

        # See: https://stackoverflow.com/a/41297449/1631514
        set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
        set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
        set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
        set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

## Configure and compile

You can enable/disable the modules that you will not need on your target system:

```
cd MRPT_SOURCE_ROOT
mkdir build-cross-arm
cmake -S. -Bbuild-cross-arm \
  -DCMAKE_TOOLCHAIN_FILE=doc/toolchain-gcc8-arm.cmake \
  -DEIGEN_USE_EMBEDDED_VERSION=ON \
  -DDISABLE_ASSIMP=ON \
  -DDISABLE_YAMLCPP=ON \
  -DDISABLE_ROS=ON \
  -DDISABLE_WXWIDGETS=ON \
  -DBUILD_APPLICATIONS=OFF \
  -DBUILD_EXAMPLES=OFF \
  -DMRPT_HAS_OPENNI2=OFF \
  -DBUILD_XSENS=OFF

# Build:
cd build-cross-arm && make
