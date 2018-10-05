# Lists of directories with source files:
#  See "DeclareMRPTLib.cmake" for explanations
# -------------------------------------------------

# Embedded version of mathplot:
if(CMAKE_MRPT_HAS_WXWIDGETS)
	list(APPEND gui_EXTRA_SRCS		"${MRPT_SOURCE_DIR}/libs/gui/src/mathplots/*.cpp" "${MRPT_SOURCE_DIR}/libs/gui/include/mrpt/otherlibs/mathplot/*.h")
	list(APPEND gui_EXTRA_SRCS_NAME 	"mathplot" "mathplot")
endif()


# Embedded freeglut?
if(CMAKE_MRPT_HAS_OPENGL_GLUT AND NOT CMAKE_MRPT_HAS_GLUT_SYSTEM)
	#if(NOT BUILD_SHARED_LIBS)
	#	list(APPEND gui_EXTRA_SRCS		"${MRPT_LIBS_ROOT}/opengl/src/freeglut-2.4.0/*.c" "${MRPT_LIBS_ROOT}/opengl/src/freeglut-2.4.0/*.h")
	#	list(APPEND gui_EXTRA_SRCS_NAME "freeglut" "freeglut")
	#endif(NOT BUILD_SHARED_LIBS)
	#add_definitions(/DFREEGLUT_STATIC)
	include_directories("${MRPT_LIBS_ROOT}/opengl/include/otherlibs/freeglut/")
endif()

# Embedded version of freeglut-2.4.0 for Windows:
# It's already in mrpt-opengl, we need its include headers:
if (WIN32)
	list(APPEND gui_EXTRA_SRCS		"${MRPT_LIBS_ROOT}/opengl/src/freeglut-2.4.0/*.h")
	list(APPEND gui_EXTRA_SRCS_NAME "freeglut-headers")
	# It's in the **opengl** lib tree:
	include_directories("${MRPT_LIBS_ROOT}/opengl/src/freeglut-2.4.0/")
endif ()

if(CMAKE_MRPT_HAS_OPENCV)
	include_directories(${OpenCV_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS} ${OPENCV_INCLUDEDIR})
endif()

#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	gui
	# Dependencies: None
	mrpt-opengl
	)

if(BUILD_mrpt-gui)
	# Ignore precompiled headers in some sources:
	if(MRPT_ENABLE_PRECOMPILED_HDRS)
		if (MSVC)
			set_source_files_properties(
				${mathplot_FILES}
				${freeglut_FILES}
				PROPERTIES COMPILE_FLAGS "/Y-")
		endif ()
	endif()

	if(CMAKE_MRPT_HAS_WXWIDGETS)
		target_link_libraries(mrpt-gui PRIVATE ${wxWidgets_LIBRARIES})
	endif()

	if(CMAKE_MRPT_HAS_Qt5)
		#qt5_use_modules(mrpt-gui Widgets OpenGL) # deprecated!
		target_link_libraries(mrpt-gui PRIVATE Qt5::Widgets Qt5::OpenGL)
	endif()

	if(CMAKE_MRPT_HAS_OPENCV)
		target_link_libraries(mrpt-gui PRIVATE ${OpenCV_LIBRARIES})
	endif()

	if(CMAKE_MRPT_HAS_OPENGL_GLUT)
		# Link against gl:
		target_link_libraries(mrpt-gui PRIVATE ${MRPT_OPENGL_LIBS})
	endif()

endif()
