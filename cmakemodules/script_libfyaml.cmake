# ===================================================
# libfyaml
# ===================================================
set(CMAKE_MRPT_HAS_LIBFYAML 0)
set(CMAKE_MRPT_HAS_LIBFYAML_SYSTEM 0)

if (WIN32)
	# libfyaml does not support Windows (as of Aug 2020)
	set(initial_has_libfyaml OFF)
else()
	set(initial_has_libfyaml ON)
endif()
OPTION(MRPT_HAS_LIBFYAML "Use libfyaml, from the system or built-in (for YAML & JSON parsing in mrpt::containers::yaml)" ${initial_has_libfyaml})
unset(initial_has_libfyaml)

if(MRPT_HAS_LIBFYAML)
	set(CMAKE_MRPT_HAS_LIBFYAML 1)
	
	# system version found?
	find_library(LIBFYAML_LIB NAMES fyaml libfyaml)
	find_path(LIBFYAML_INCLUDES NAMES libfyaml.h)
	if (LIBFYAML_LIB AND LIBFYAML_INCLUDES)
		set(CMAKE_MRPT_HAS_LIBFYAML_SYSTEM 1)
		if ($ENV{VERBOSE})
			message(STATUS "LIBFYAML_LIB      : ${LIBFYAML_LIB}")
			message(STATUS "LIBFYAML_INCLUDES : ${LIBFYAML_INCLUDES}")
		endif()
	else()
		# Internal built-in:
		include(ExternalProject)
		
		set(LIBFYAML_DIR ${MRPT_SOURCE_DIR}/3rdparty/libfyaml)
		set(LIBFYAML_BIN ${MRPT_BINARY_DIR}/3rdparty/libfyaml)
		set(LIBFYAML_INSTALL_DIR ${LIBFYAML_BIN}/install)
		set(LIBFYAML_INCLUDES ${LIBFYAML_DIR}/include)
		
		set(LIBFYAML_LIB ${LIBFYAML_INSTALL_DIR}/${CMAKE_INSTALL_LIBDIR}/${CMAKE_STATIC_LIBRARY_PREFIX}fyaml${CMAKE_STATIC_LIBRARY_SUFFIX})
		
		ExternalProject_Add(
			mrpt_liblibfyaml
			PREFIX ${LIBFYAML_BIN}
			SOURCE_DIR ${LIBFYAML_DIR}
			INSTALL_DIR "${LIBFYAML_INSTALL_DIR}"
			CMAKE_ARGS
			-DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
			-DBUILD_SHARED_LIBS:BOOL=OFF
			-DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
			-DCMAKE_INSTALL_LIBDIR=${CMAKE_INSTALL_LIBDIR}
			-DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
			-DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
			BUILD_BYPRODUCTS ${LIBFYAML_LIB}
			)
	endif()
		
	if (NOT CMAKE_MRPT_HAS_LIBFYAML_SYSTEM)
		add_library(mrpt_libfyaml STATIC IMPORTED GLOBAL)
		add_dependencies(mrpt_libfyaml mrpt_liblibfyaml)
		set_target_properties(mrpt_libfyaml PROPERTIES IMPORTED_LOCATION ${LIBFYAML_LIB})
		set_target_properties(mrpt_libfyaml PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${LIBFYAML_INCLUDES})
	else()
		add_library(mrpt_libfyaml INTERFACE)
		target_include_directories(mrpt_libfyaml SYSTEM INTERFACE ${LIBFYAML_INCLUDES})
		target_link_libraries(mrpt_libfyaml INTERFACE ${LIBFYAML_LIB})
	endif()

endif()
