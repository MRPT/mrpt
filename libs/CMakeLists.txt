project(ALL_MRPT_LIBS)

# ------------------------------------------
# Enumerate al subdirectories under "libs/"
#  and process its CMake files: 
# ------------------------------------------
list_subdirectories(LIST_MRPT_LIB_DIRS "${MRPT_SOURCE_DIR}/libs")

foreach(dir ${LIST_MRPT_LIB_DIRS})
  add_subdirectory(${dir})
endforeach(dir)
