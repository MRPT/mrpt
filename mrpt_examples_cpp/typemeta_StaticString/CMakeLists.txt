#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example: typemeta_StaticString
# Generated automatically - Manual changes may be overwritten.
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(typemeta_StaticString)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_typemeta REQUIRED)


# Define the executable target:
mrpt_add_executable(
  TARGET typemeta_StaticString
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_typemeta 
)
