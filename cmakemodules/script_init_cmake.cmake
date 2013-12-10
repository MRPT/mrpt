if(COMMAND cmake_policy)
	# We don't want to mix relative and absolute paths in linker lib lists.
	cmake_policy(SET CMP0003 NEW) 
	
	# Allow using the LOCATION target property.
	if(NOT "${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 3.0)
		cmake_policy(SET CMP0026 OLD)
	endif(NOT "${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 3.0)
endif(COMMAND cmake_policy)
