#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example: nav_ptg_tpspace
# Generated automatically - Manual changes may be overwritten.
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(nav_ptg_tpspace)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_nav REQUIRED)

# Define the executable target:
mrpt_add_executable(
  TARGET nav_ptg_tpspace
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_nav
)
