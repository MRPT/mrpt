if(COMMAND cmake_policy)
	# We don't want to mix relative and absolute paths in linker lib lists.
	cmake_policy(SET CMP0003 NEW) 
	
#	if(POLICY CMP0054)
#		cmake_policy(SET CMP0054 NEW) # http://www.cmake.org/cmake/help/v3.1/policy/CMP0054.html
#	endif(POLICY CMP0054)
endif(COMMAND cmake_policy)
