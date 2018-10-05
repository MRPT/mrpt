# Check for the sys/inotify headers (Linux only, in win32
#  we use the equivalent API for file system monitoring):
# =======================================================
if(UNIX)
	find_file(INOTIFY_HEADER_FILE sys/inotify.h HINTS /usr/include/x86_64-linux-gnu/)
	if(INOTIFY_HEADER_FILE)
		set(CMAKE_MRPT_HAS_INOTIFY 1)
		mark_as_advanced(INOTIFY_HEADER_FILE)
	else(INOTIFY_HEADER_FILE)
		set(CMAKE_MRPT_HAS_INOTIFY 0)
	endif(INOTIFY_HEADER_FILE)
else(UNIX)
	# In windows there is no INOTIFY api.
	set(CMAKE_MRPT_HAS_INOTIFY 0)
endif(UNIX)
