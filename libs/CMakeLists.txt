project(ALL_MRPT_LIBS)

# -----------------------------------------------
# 1) Enumerate al subdirectories under "libs/"
# -----------------------------------------------
list_subdirectories(LIST_MRPT_LIB_DIRS "${MRPT_SOURCE_DIR}/libs")

# -----------------------------------------------
# 2) Auto detect which libs are header-only:
# -----------------------------------------------
foreach(dir ${LIST_MRPT_LIB_DIRS})
	if (EXISTS "${dir}/include/" AND NOT EXISTS "${dir}/src/")
		set_property(GLOBAL PROPERTY "mrpt-${dir}_LIB_IS_HEADERS_ONLY" "1")
	endif (EXISTS "${dir}/include/" AND NOT EXISTS "${dir}/src/")
endforeach(dir)

# -----------------------------------------------
# 3) Enumerate al subdirectories under "libs/"
# -----------------------------------------------
foreach(dir ${LIST_MRPT_LIB_DIRS})
  add_subdirectory(${dir})
endforeach(dir)


