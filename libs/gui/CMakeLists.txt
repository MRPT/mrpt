# Lists of directories with source files:
#  See "DeclareMRPTLib.cmake" for explanations
# -------------------------------------------------

# Embedded version of mathplot:
if(CMAKE_MRPT_HAS_WXWIDGETS)
	list(APPEND gui_EXTRA_SRCS		"${MRPT_SOURCE_DIR}/libs/gui/src/mathplots/*.cpp" "${MRPT_SOURCE_DIR}/libs/gui/include/mrpt/3rdparty/mathplot/*.h")
	list(APPEND gui_EXTRA_SRCS_NAME 	"mathplot" "mathplot")
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
				PROPERTIES COMPILE_FLAGS "/Y-")
		endif ()
	endif()

	if(CMAKE_MRPT_HAS_WXWIDGETS)
		if (NOT TARGET imp_wxwidgets)
			message(ERROR "imp_wxwidgets should be defined!")
		endif()
		target_link_libraries(gui PRIVATE imp_wxwidgets)
	endif()

	if(CMAKE_MRPT_HAS_Qt5)
		#qt5_use_modules(mrpt-gui Widgets OpenGL) # deprecated!
		target_link_libraries(gui PRIVATE Qt5::Widgets Qt5::OpenGL)
	endif()

	if(CMAKE_MRPT_HAS_OPENCV)
		target_link_libraries(gui PRIVATE imp_opencv)
	endif()

	if(MRPT_OPENGL_LIBS)
		# Link against glut:
		target_link_libraries(gui PRIVATE ${MRPT_OPENGL_LIBS})
	endif()

	if (TARGET mrpt_glew) # Basically, for Windows
		target_link_libraries(gui PRIVATE mrpt_glew)
	endif()

endif()
