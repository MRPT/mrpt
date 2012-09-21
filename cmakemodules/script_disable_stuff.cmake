# Let the user disable SSE2
# ===================================================
SET(DISABLE_SSE2 OFF CACHE BOOL "Forces compilation WITHOUT SSE2/MMX extensions")
MARK_AS_ADVANCED(DISABLE_SSE2)
IF(DISABLE_SSE2)
	SET(CMAKE_MRPT_HAS_SSE2 0)
ENDIF(DISABLE_SSE2)

# Let the user disable SSE3
# ===================================================
SET(DISABLE_SSE3 OFF CACHE BOOL "Forces compilation WITHOUT SSE3 extensions")
MARK_AS_ADVANCED(DISABLE_SSE3)
IF(DISABLE_SSE3)
	SET(CMAKE_MRPT_HAS_SSE3 0)
ENDIF(DISABLE_SSE3)

# Let the user disable SSE4
# ===================================================
SET(DISABLE_SSE4 OFF CACHE BOOL "Forces compilation WITHOUT SSE4 extensions")
MARK_AS_ADVANCED(DISABLE_SSE4)
IF(DISABLE_SSE4)
	SET(CMAKE_MRPT_HAS_SSE4 0)
ENDIF(DISABLE_SSE4)

# DISABLE_OPENGL
# ---------------------
OPTION(DISABLE_OPENGL "Disable the OpenGL library" "OFF")
MARK_AS_ADVANCED(DISABLE_OPENGL)
IF(DISABLE_OPENGL)
	SET(CMAKE_MRPT_HAS_OPENGL_GLUT 0)
ENDIF(DISABLE_OPENGL)

