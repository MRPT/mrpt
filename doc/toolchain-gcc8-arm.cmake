# Save as toolchain.cmake
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

