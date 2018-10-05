project( mexplus )

# Code below is just for accessibility issues in IDE editor
# List all files (headers) contained by mexplus library
file(GLOB_RECURSE all_mexplus_files RELATIVE "${CMAKE_SOURCE_DIR}/otherlibs/mexplus" *.h *.hpp)
# Visualize the files of this directory in IDE creating an custom empty target
add_custom_target(mexplus DEPENDS ${all_mexplus_files} SOURCES ${all_mexplus_files})
