# Lists of directories with source files:
#  See "DeclareMRPTLib.cmake" for explanations
# -------------------------------------------------

# Embedded version of mathplot:
if(CMAKE_MRPT_HAS_WXWIDGETS)
	list(APPEND gui_EXTRA_SRCS		"${MRPT_SOURCE_DIR}/libs/gui/src/mathplots/*.cpp" "${MRPT_SOURCE_DIR}/libs/gui/include/mrpt/3rdparty/mathplot/*.h")
	list(APPEND gui_EXTRA_SRCS_NAME 	"mathplot" "mathplot")
endif()

if (CMAKE_MRPT_HAS_NANOGUI)
	set(nanogui_dep mrpt-nanogui)
else()
	set(nanogui_dep "")
endif()

#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	gui
	# Dependencies: None
	mrpt-opengl
	# Other imported targets:
	${nanogui_dep}
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

	if (CMAKE_MRPT_HAS_NANOGUI)
		target_link_libraries(gui PUBLIC mrpt::nanogui)
	endif()

	# Use the mrpt::opengl profiler option here too for the opengl GUIs:
	if (MRPT_OPENGL_PROFILER)
		target_compile_definitions(gui PRIVATE MRPT_OPENGL_PROFILER)
	endif()
endif()
