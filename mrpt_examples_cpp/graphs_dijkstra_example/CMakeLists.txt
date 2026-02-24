#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example: graphs_dijkstra_example
# Generated automatically - Manual changes may be overwritten.
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(graphs_dijkstra_example)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_graphs REQUIRED)
find_package(mrpt_gui REQUIRED)


# Define the executable target:
mrpt_add_executable(
  TARGET graphs_dijkstra_example
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_graphs mrpt::mrpt_gui 
)
