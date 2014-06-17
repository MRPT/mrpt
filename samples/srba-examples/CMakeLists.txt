PROJECT(ALL_SRBA_EXAMPLES)

# Set the output directory of all srba examples to this subdirectory in the binary tree:
SET(EXECUTABLE_OUTPUT_PATH ".")

if(BUILD_mrpt-srba)
	add_subdirectory(srba-tutorials)
endif(BUILD_mrpt-srba)
