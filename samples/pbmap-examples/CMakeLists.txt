PROJECT(ALL_PBMAP_EXAMPLES)

# Set the output directory of all pbmap examples to this subdirectory in the binary tree:
SET(EXECUTABLE_OUTPUT_PATH ".")

if(BUILD_mrpt-pbmap)
	add_subdirectory(pbmap-tutorials)
endif(BUILD_mrpt-pbmap)
