# Declares the dependencies of an application:
# Usage: DeclareAppDependencies( appTargetName [mrpt-xxx [mrpt-yyy] ...] )
#
macro(DeclareAppDependencies name)
	set(ALL_DEPS "")
	
	FOREACH(DEP ${ARGN})
		LIST(APPEND ALL_DEPS ${DEP})
	ENDFOREACH(DEP)

	FOREACH(DEP ${ARGN})
		# Only for "mrpt-XXX" libs:
		IF (${DEP} MATCHES "mrpt-")
			get_property(LIB_DEP GLOBAL PROPERTY "${DEP}_LIB_DEPS")
			LIST(APPEND ALL_DEPS ${LIB_DEP})
		ENDIF (${DEP} MATCHES "mrpt-")
	ENDFOREACH(DEP)
	
	
	# Add the detected dependencies:
	list(REMOVE_DUPLICATES ALL_DEPS)
	IF (NOT "${ALL_DEPS}" STREQUAL "")
		#MESSAGE(STATUS "Adding deps: ${name} --> ${ALL_DEPS}")
		ADD_DEPENDENCIES(${name} ${ALL_DEPS})
		
		FOREACH (_DEP ${ALL_DEPS})
			# Link:
			IF(CMAKE_COMPILER_IS_GNUCXX)
				TARGET_LINK_LIBRARIES(${name} ${_DEP}${MRPT_LINKER_LIBS_POSTFIX})
			ENDIF(CMAKE_COMPILER_IS_GNUCXX)
			
			# Include:
			STRING(REGEX REPLACE "mrpt-(.*)" "\\1" DEP_MRPT_NAME ${_DEP})
			IF(NOT "${DEP_MRPT_NAME}" STREQUAL "")
				INCLUDE_DIRECTORIES("${MRPT_LIBS_ROOT}/${DEP_MRPT_NAME}/include/")
			ENDIF(NOT "${DEP_MRPT_NAME}" STREQUAL "")
		ENDFOREACH (_DEP)		
	ENDIF (NOT "${ALL_DEPS}" STREQUAL "")
	
endmacro(DeclareAppDependencies)
