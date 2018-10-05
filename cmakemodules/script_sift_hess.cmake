# By default, compile this library if the directory exist:
# It will be not present in Debian packages due to legal restrictions:
# ------------------------------------------------------------------------
if(ALLOW_GPL)
	if(EXISTS "${MRPT_LIBS_ROOT}/vision/src/sift-hess")
		set( CMAKE_MRPT_HAS_SIFT_HESS 1)
	else(EXISTS "${MRPT_LIBS_ROOT}/vision/src/sift-hess")
		set( CMAKE_MRPT_HAS_SIFT_HESS 0)
	endif(EXISTS "${MRPT_LIBS_ROOT}/vision/src/sift-hess")

	if(NOT CMAKE_MRPT_HAS_OPENCV)
		set( CMAKE_MRPT_HAS_SIFT_HESS 0)
	endif(NOT CMAKE_MRPT_HAS_OPENCV)

	option(DISABLE_SIFT_HESS "Disable the Hess' SIFT library" "OFF")
	mark_as_advanced(DISABLE_SIFT_HESS)
	if(DISABLE_SIFT_HESS)
		set(CMAKE_MRPT_HAS_SIFT_HESS 0)
	endif(DISABLE_SIFT_HESS)
else(ALLOW_GPL)
	set( CMAKE_MRPT_HAS_SIFT_HESS 0)
endif(ALLOW_GPL)
