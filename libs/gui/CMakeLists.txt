# Lists of directories with source files:
#  See "DeclareMRPTLib.cmake" for explanations
# -------------------------------------------------

# Embedded version of mathplot:
IF(CMAKE_MRPT_HAS_WXWIDGETS)
	LIST(APPEND gui_EXTRA_SRCS		"${MRPT_SOURCE_DIR}/libs/gui/src/mathplots/*.cpp" "${MRPT_SOURCE_DIR}/libs/gui/include/mrpt/otherlibs/mathplot/*.h")
	LIST(APPEND gui_EXTRA_SRCS_NAME 	"mathplot" "mathplot")
ENDIF()


# Embedded freeglut?
IF(CMAKE_MRPT_HAS_OPENGL_GLUT AND NOT CMAKE_MRPT_HAS_GLUT_SYSTEM)
	#IF(NOT BUILD_SHARED_LIBS)
	#	LIST(APPEND gui_EXTRA_SRCS		"${MRPT_LIBS_ROOT}/opengl/src/freeglut-2.4.0/*.c" "${MRPT_LIBS_ROOT}/opengl/src/freeglut-2.4.0/*.h")
	#	LIST(APPEND gui_EXTRA_SRCS_NAME "freeglut" "freeglut")
	#ENDIF(NOT BUILD_SHARED_LIBS)
	#ADD_DEFINITIONS(/DFREEGLUT_STATIC)
	INCLUDE_DIRECTORIES("${MRPT_LIBS_ROOT}/opengl/include/otherlibs/freeglut/")
ENDIF()

# Embedded version of freeglut-2.4.0 for Windows:
# It's already in mrpt-opengl, we need its include headers:
IF (WIN32)
	LIST(APPEND gui_EXTRA_SRCS		"${MRPT_LIBS_ROOT}/opengl/src/freeglut-2.4.0/*.h")
	LIST(APPEND gui_EXTRA_SRCS_NAME "freeglut-headers")
	# It's in the **opengl** lib tree:
	include_directories("${MRPT_LIBS_ROOT}/opengl/src/freeglut-2.4.0/")
ENDIF ()

IF(CMAKE_MRPT_HAS_OPENCV)
	INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS} ${OPENCV_INCLUDEDIR})
ENDIF()


#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	gui
	# Dependencies: None
	mrpt-opengl
	mrpt-base
	)

IF(BUILD_mrpt-gui)
	# Ignore precompiled headers in some sources:
	IF(MRPT_ENABLE_PRECOMPILED_HDRS)
		IF (MSVC)
			set_source_files_properties(
				${mathplot_FILES}
				${freeglut_FILES}
				PROPERTIES COMPILE_FLAGS "/Y-")
		ENDIF ()
	ENDIF()

	IF(CMAKE_MRPT_HAS_WXWIDGETS)
		target_link_libraries(mrpt-gui ${wxWidgets_LIBRARIES})
	ENDIF()

	IF(CMAKE_MRPT_HAS_OPENCV)
		target_link_libraries(mrpt-gui ${OpenCV_LIBRARIES})
	ENDIF()
ENDIF()
