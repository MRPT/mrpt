include(ExternalProject)

ExternalProject_Add(libcvd
	SOURCE_DIR ${MRPT_SOURCE_DIR}/otherlibs/libcvd
	CMAKE_ARGS
		-DCMAKE_POSITION_INDEPENDENT_CODE=ON
		# No SSE2: Gives problems in non Intel archs, and anyway we don't exploit libcvd CPU-intensive ops from mrpt
		-DCVD_SSE2=OFF
		-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
		-DCVD_TESTS=OFF
		-DCVD_PROGS=OFF
		-DCVD_EXAMPLES=OFF
	INSTALL_COMMAND ""
)

set(CVD_DIR "${MRPT_SOURCE_DIR}/otherlibs/libcvd")
set(CVD_BIN_DIR "${MRPT_BINARY_DIR}/libcvd-prefix/src/libcvd-build")

set(CVD_INCLUDE_DIRS "${CVD_DIR}" "${CVD_BIN_DIR}/include")

if(MSVC)
	set(CVD_LIB "${CVD_BIN_DIR}/$<CONFIG>/CVD$<$<CONFIG:Debug>:_debug>.lib")
else()
	set(CVD_LIB "${CVD_BIN_DIR}/libCVD$<$<CONFIG:Debug>:_debug>.a")
endif()
