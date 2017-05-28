cmake_minimum_required(VERSION 2.8.11)

project(benchmarking-image-features)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

# Find the QtWidgets library
find_package(Qt5Widgets)

# Tell CMake to create the benchmarking executable
add_executable(benchmarking_GUI main.cpp mainwindow.h mainwindow.cpp)

# Use the Widgets module from Qt 5.
target_link_libraries(benchmarking_GUI Qt5::Widgets)
