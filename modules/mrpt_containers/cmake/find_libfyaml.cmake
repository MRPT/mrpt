# ===================================================
# libfyaml
# ===================================================
set(CMAKE_MRPT_HAS_LIBFYAML 0)
set(CMAKE_MRPT_HAS_LIBFYAML_SYSTEM 0)

OPTION(MRPT_HAS_LIBFYAML "Use libfyaml, from the system or built-in (for YAML & JSON parsing in mrpt::containers::yaml)" ON)

if(MRPT_HAS_LIBFYAML)
	set(CMAKE_MRPT_HAS_LIBFYAML 1)
	
	# system version found?
	find_package(PkgConfig QUIET)
	if (PkgConfig_FOUND)
		pkg_check_modules(LIBFYAML IMPORTED_TARGET libfyaml)
		if (LIBFYAML_FOUND)
			set(CMAKE_MRPT_HAS_LIBFYAML_SYSTEM 1)
		endif()
	endif()

	if (NOT CMAKE_MRPT_HAS_LIBFYAML_SYSTEM)
		# Internal built-in:
		include(ExternalProject)

		set(LIBFYAML_DIR ${mrpt_containers_SOURCE_DIR}/3rdparty/libfyaml)

		# Check that the git submodule has been initialized
		if(NOT EXISTS "${LIBFYAML_DIR}/CMakeLists.txt")
			message(FATAL_ERROR
				"The libfyaml git submodule is missing or not initialized.\n"
				"Please run:\n"
				"  git submodule update --init --recursive\n"
				"from the root of the repository and then rebuild."
			)
		endif()
		set(LIBFYAML_BIN ${mrpt_containers_BINARY_DIR}/3rdparty/libfyaml)
		set(LIBFYAML_INSTALL_DIR ${LIBFYAML_BIN}/install)
		set(LIBFYAML_INCLUDES ${LIBFYAML_DIR}/include)
		
		# On Windows with MSVC, the library installs as fyaml.lib (no prefix),
		# regardless of CMAKE_STATIC_LIBRARY_PREFIX which MRPT overrides to "lib".
		if(WIN32 AND MSVC)
			set(LIBFYAML_LIB ${LIBFYAML_INSTALL_DIR}/${CMAKE_INSTALL_LIBDIR}/fyaml${CMAKE_STATIC_LIBRARY_SUFFIX})
		else()
			set(LIBFYAML_LIB ${LIBFYAML_INSTALL_DIR}/${CMAKE_INSTALL_LIBDIR}/${CMAKE_STATIC_LIBRARY_PREFIX}fyaml${CMAKE_STATIC_LIBRARY_SUFFIX})
		endif()
		
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
			-DENABLE_LIBCLANG=OFF
			-DCMAKE_POSITION_INDEPENDENT_CODE=ON
			BUILD_BYPRODUCTS ${LIBFYAML_LIB}
			)
		file(READ "${LIBFYAML_DIR}/.tarball-version" LIBFYAML_VERSION)
		string(STRIP "${LIBFYAML_VERSION}" LIBFYAML_VERSION)
	endif()

	if (CMAKE_MRPT_HAS_LIBFYAML_SYSTEM)
		add_library(mrpt_libfyaml ALIAS PkgConfig::LIBFYAML)
	elseif(CMAKE_MRPT_HAS_LIBFYAML)
		add_library(mrpt_libfyaml STATIC IMPORTED GLOBAL)
		add_dependencies(mrpt_libfyaml mrpt_liblibfyaml)
		set_target_properties(mrpt_libfyaml PROPERTIES IMPORTED_LOCATION ${LIBFYAML_LIB})
		set_target_properties(mrpt_libfyaml PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${LIBFYAML_INCLUDES})
	endif()

	if ($ENV{VERBOSE})
			message(STATUS "LIBFYAML_VERSION  : ${LIBFYAML_VERSION}")
	endif()
endif()
