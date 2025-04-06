project(ALL_MRPT_LIBS)

set(DISABLE_MRPT_AUTO_CLASS_REGISTRATION OFF CACHE BOOL "Enable to reduce the footprint of MRPT statically libraries linked; caution: all deserialized classes must be registered by hand by the user!")
mark_as_advanced(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
if(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
	add_definitions(-DDISABLE_MRPT_AUTO_CLASS_REGISTRATION)
endif()


# -----------------------------------------------
# 1) Enumerate al subdirectories under "libs/"
# -----------------------------------------------
list_subdirectories(LIST_MRPT_LIB_DIRS "${MRPT_SOURCE_DIR}/libs")

# -----------------------------------------------
# 2) Auto detect which libs are header-only:
# -----------------------------------------------
foreach(dir ${LIST_MRPT_LIB_DIRS})
	# if "define_mrpt_lib_header_only()" is used, it's a hdr-only library.
	file(READ "${MRPT_SOURCE_DIR}/libs/${dir}/CMakeLists.txt" TMP)
	string(FIND "${TMP}" "define_mrpt_lib_header_only" fndpos)
	if(${fndpos} EQUAL -1)
		set(is_hdr "0")
	else()
		set(is_hdr "1")
	endif()
	set_property(GLOBAL PROPERTY "mrpt-${dir}_LIB_IS_HEADERS_ONLY" "${is_hdr}")
	#message(STATUS "mrpt-${dir}_LIB_IS_HEADERS_ONLY = ${is_hdr}")
endforeach(dir)

# -----------------------------------------------
# 3) Enumerate al subdirectories under "libs/"
# -----------------------------------------------
foreach(dir ${LIST_MRPT_LIB_DIRS})
  add_subdirectory(${dir})
endforeach(dir)


