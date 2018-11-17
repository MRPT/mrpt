/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
 */

/** \page mrpt_from_cmake Using MRPT from your CMake project

## Finding MRPT from CMake

MRPT defines exported projects that can be imported as usual in modern CMake:

```
# Find MRPT libraries:
find_package(mrpt-poses)
find_package(mrpt-gui)

# Define your own targets:
add_executable(myapp  main.cpp)

# Link against MRPT: this will also add all required flags,
# include directories, etc.
target_link_libraries(myapp
  mrpt-poses
  mrpt-gui
)
```

## For MRPT 1.x

Prior to MRPT 2.0.0, the correct way to find for MRPT was different:

```
# Find MRPT libraries:
find_package(MRPT REQUIRED poses gui)

add_executable(myapp  main.cpp)
target_link_libraries(myapp ${MRPT_LIBS})
```


 */
