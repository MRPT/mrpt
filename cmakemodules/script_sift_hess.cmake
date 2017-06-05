# By default, compile this library if the directory exist:
# It will be not present in Debian packages due to legal restrictions:
# ------------------------------------------------------------------------
IF(ALLOW_GPL)
	IF(EXISTS "${MRPT_LIBS_ROOT}/vision/src/sift-hess")
		SET( CMAKE_MRPT_HAS_SIFT_HESS 1)
	ELSE(EXISTS "${MRPT_LIBS_ROOT}/vision/src/sift-hess")
		SET( CMAKE_MRPT_HAS_SIFT_HESS 0)
	ENDIF(EXISTS "${MRPT_LIBS_ROOT}/vision/src/sift-hess")

	IF(NOT CMAKE_MRPT_HAS_OPENCV)
		SET( CMAKE_MRPT_HAS_SIFT_HESS 0)
	ENDIF(NOT CMAKE_MRPT_HAS_OPENCV)

	OPTION(DISABLE_SIFT_HESS "Disable the Hess' SIFT library" "OFF")
	MARK_AS_ADVANCED(DISABLE_SIFT_HESS)
	IF(DISABLE_SIFT_HESS)
		SET(CMAKE_MRPT_HAS_SIFT_HESS 0)
	ENDIF(DISABLE_SIFT_HESS)
ELSE(ALLOW_GPL)
	SET( CMAKE_MRPT_HAS_SIFT_HESS 0)
ENDIF(ALLOW_GPL)
