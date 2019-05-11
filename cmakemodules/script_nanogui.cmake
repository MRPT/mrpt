# Declares external project: nanogui
#  https://github.com/wjakob/nanogui
# ===================================================
set(CMAKE_MRPT_HAS_NANOGUI 0)
set(CMAKE_MRPT_HAS_NANOGUI_SYSTEM 0)

set(EMBEDDED_NANOGUI_DIR "${MRPT_BINARY_DIR}/otherlibs/nanogui")

set(BUILD_NANOGUI ON CACHE BOOL "Build an embedded version of nanogui (OpenGL GUIs)")
if (BUILD_NANOGUI)
	include(ExternalProject)

	set(nanogui_PREFIX "${MRPT_BINARY_DIR}/otherlibs/nanogui")
	set(nanogui_INSTALL_DIR "${MRPT_BINARY_DIR}/otherlibs/nanogui/install")
	set(nanogui_CMAKE_ARGS
		-DCMAKE_INSTALL_PREFIX=${nanogui_INSTALL_DIR}
		-DNANOGUI_BUILD_PYTHON=OFF
		-DNANOGUI_BUILD_EXAMPLE=OFF
		-DCMAKE_LIBRARY_OUTPUT_PATH=${MRPT_BINARY_DIR}/lib
		-DLIBRARY_OUTPUT_PATH=${MRPT_BINARY_DIR}/lib
		-DCMAKE_RUNTIME_OUTPUT_DIRECTORY=${MRPT_BINARY_DIR}/bin
		-DRUNTIME_OUTPUT_DIRECTORY=${MRPT_BINARY_DIR}/bin
		-DCMAKE_DEBUG_POSTFIX=${CMAKE_DEBUG_POSTFIX}
	)

	# download from GH
	ExternalProject_Add(EP_nanogui
		PREFIX ${nanogui_PREFIX}
		GIT_REPOSITORY https://github.com/wjakob/nanogui.git
		GIT_SUBMODULES ext/eigen ext/glfw ext/nanovg # ext/pybind
		INSTALL_DIR ${nanogui_INSTALL_DIR}
		CMAKE_ARGS ${nanogui_CMAKE_ARGS}
	)
	set(CMAKE_MRPT_HAS_NANOGUI 1)
	set(CMAKE_MRPT_HAS_NANOGUI_SYSTEM 0)
endif()

if (CMAKE_MRPT_HAS_ASSIMP)
	if (NOT "${ASSIMP_LIBRARY_DIRS}" STREQUAL "")
		link_directories("${ASSIMP_LIBRARY_DIRS}")
	endif()

	mark_as_advanced(ASSIMP_DIR)

	if ($ENV{VERBOSE})
		message(STATUS "Assimp:")
		message(STATUS " ASSIMP_INCLUDE_DIRS: ${ASSIMP_INCLUDE_DIRS}")
		message(STATUS " ASSIMP_CXX_FLAGS: ${ASSIMP_CXX_FLAGS}")
		message(STATUS " ASSIMP_LINK_FLAGS: ${ASSIMP_LINK_FLAGS}")
		message(STATUS " ASSIMP_LIBRARIES: ${ASSIMP_LIBRARIES}")
		message(STATUS " ASSIMP_LIBRARY_DIRS: ${ASSIMP_LIBRARY_DIRS}")
		message(STATUS " ASSIMP_VERSION: ${ASSIMP_VERSION}")
	endif ($ENV{VERBOSE})
endif (CMAKE_MRPT_HAS_ASSIMP)
