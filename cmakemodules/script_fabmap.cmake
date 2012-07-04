#   FabMap module for HMT-SLAM? (optional)
# =======================================
FIND_PACKAGE(FabMap QUIET)
IF(NOT FabMap_DIR)
	MARK_AS_ADVANCED(FabMap_DIR)
ENDIF(NOT FabMap_DIR)

IF(FabMap_FOUND)
	SET(CMAKE_MRPT_HAS_FABMAP  1)
ELSE(FabMap_FOUND)
	SET(CMAKE_MRPT_HAS_FABMAP  0)
ENDIF(FabMap_FOUND)
