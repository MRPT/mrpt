# Check for the sys/inotify headers (Linux only, in win32
#  we use the equivalent API for file system monitoring):
# =======================================================
IF(UNIX)
	FIND_FILE(INOTIFY_HEADER_FILE sys/inotify.h HINTS /usr/include/x86_64-linux-gnu/)
	IF(INOTIFY_HEADER_FILE)
		SET(CMAKE_MRPT_HAS_INOTIFY 1)
		MARK_AS_ADVANCED(INOTIFY_HEADER_FILE)
	ELSE(INOTIFY_HEADER_FILE)
		SET(CMAKE_MRPT_HAS_INOTIFY 0)
	ENDIF(INOTIFY_HEADER_FILE)
ELSE(UNIX)
	# In windows there is no INOTIFY api.
	SET(CMAKE_MRPT_HAS_INOTIFY 0)
ENDIF(UNIX)
