project(ALL_MRPT_LIBS)

SET(DISABLE_MRPT_AUTO_CLASS_REGISTRATION OFF CACHE BOOL "Enable to reduce the footprint of MRPT statically libraries linked; caution: all deserialized classes must be registered by hand by the user!")
MARK_AS_ADVANCED(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
IF(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
	ADD_DEFINITIONS(-DDISABLE_MRPT_AUTO_CLASS_REGISTRATION)
ENDIF()


# -----------------------------------------------
# 1) Enumerate al subdirectories under "libs/"
# -----------------------------------------------
list_subdirectories(LIST_MRPT_LIB_DIRS "${MRPT_SOURCE_DIR}/libs")

# -----------------------------------------------
# 2) Auto detect which libs are header-only:
# -----------------------------------------------
foreach(dir ${LIST_MRPT_LIB_DIRS})
	set(is_hdr "0")
	if (EXISTS "${MRPT_SOURCE_DIR}/libs/${dir}/include/mrpt/${dir}.h" AND NOT EXISTS "${MRPT_SOURCE_DIR}/libs/${dir}/src/registerAllClasses.cpp")
		set(is_hdr "1")
	endif (EXISTS "${MRPT_SOURCE_DIR}/libs/${dir}/include/mrpt/${dir}.h" AND NOT EXISTS "${MRPT_SOURCE_DIR}/libs/${dir}/src/registerAllClasses.cpp")

	set_property(GLOBAL PROPERTY "mrpt-${dir}_LIB_IS_HEADERS_ONLY" "${is_hdr}")
	#MESSAGE(STATUS "mrpt-${dir}_LIB_IS_HEADERS_ONLY = ${is_hdr}")
endforeach(dir)

# -----------------------------------------------
# 3) Enumerate al subdirectories under "libs/"
# -----------------------------------------------
foreach(dir ${LIST_MRPT_LIB_DIRS})
  add_subdirectory(${dir})
endforeach(dir)


