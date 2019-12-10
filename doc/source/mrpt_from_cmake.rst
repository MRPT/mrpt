.. _mrpt_from_cmake:

####################################
Using MRPT from your CMake project
####################################


Finding MRPT from CMake
-------------------------

MRPT defines exported projects that can be imported as usual in modern CMake:

```
# Find all MRPT libraries:
find_package(MRPT 1.9.9 COMPONENTS poses gui OPTIONAL_COMPONENTS vision)
message(STATUS "MRPT_VERSION: ${MRPT_VERSION}")
message(STATUS "MRPT_LIBRARIES: ${MRPT_LIBRARIES}")

# Define your own targets:
add_executable(myapp  main.cpp)

# Link against MRPT: this will also add all required flags,
# include directories, etc.
target_link_libraries(myapp ${MRPT_LIBRARIES})
```
or individually like:

```
# Find MRPT libraries, one by one:
find_package(mrpt-poses)
find_package(mrpt-gui)

# Define your own targets:
add_executable(myapp  main.cpp)

# Link against MRPT: this will also add all required flags,
# include directories, etc.
target_link_libraries(myapp
  mrpt::poses
  mrpt::gui
)
```


For MRPT 1.x
-------------------------

Prior to MRPT 2.0.0, the correct way to find for MRPT was:
```
# Find MRPT libraries:
find_package(MRPT REQUIRED poses gui)

add_executable(myapp  main.cpp)
target_link_libraries(myapp ${MRPT_LIBRARIES})
```
