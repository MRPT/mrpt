include(ExternalProject)

ExternalProject_Add(libcvd
	SOURCE_DIR ${MRPT_SOURCE_DIR}/otherlibs/libcvd
	CMAKE_ARGS
		-DCMAKE_POSITION_INDEPENDENT_CODE=ON
		-DCVD_SSE2=${CMAKE_MRPT_HAS_SSE2}
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
	set(CVD_LIB "${CVD_BIN_DIR}/libCVD.a")
endif()
