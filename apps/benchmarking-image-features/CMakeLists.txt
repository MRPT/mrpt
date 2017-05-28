project(benchmarkingImageFeatures_GUI)
cmake_minimum_required(VERSION 2.8.11)


# include(../../cmakemodules/script_qt.cmake)

project(benchmarking-image-features)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

set(CPP_FILES main.cpp mainwindow.h mainwindow.cpp)

# Find the QtWidgets library
find_package(Qt5Widgets)

# Find the MRPT packages
find_package(MRPT REQUIRED base;gui)

# Tell CMake to create the benchmarking executable
add_executable(${PROJECT_NAME} ${CPP_FILES})

# Use the Widgets module from Qt 5.
target_link_libraries(${PROJECT_NAME} Qt5::Widgets ${MRPT_LIBS})
