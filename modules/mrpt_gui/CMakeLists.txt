
cmake_minimum_required(VERSION 3.16)
project(mrpt_gui LANGUAGES C CXX)

# Ensure core MRPT packages used here are available to CMake
find_package(mrpt_opengl REQUIRED)

include(./cmake/script_nanogui.cmake REQUIRED)          # Check for wxWidgets + GL
include(./cmake/script_qt.cmake REQUIRED)          # Check for wxWidgets + GL
include(./cmake/script_wxwidgets.cmake REQUIRED)   # Check for wxWidgets + GL

find_package(mrpt_nanogui)

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

# define lib:
set(LIB_SOURCES "")
set(LIB_PUBLIC_HEADERS "")

if(CMAKE_MRPT_HAS_WXWIDGETS)
  list(APPEND LIB_SOURCES ${gui_EXTRA_SRCS})
  list(APPEND LIB_PUBLIC_HEADERS ${gui_EXTRA_SRCS})
endif()

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_opengl
    ${nanogui_dep}
  CMAKE_DEPENDENCIES
    mrpt_opengl
)

if(BUILD_mrpt-gui)
	if(CMAKE_MRPT_HAS_WXWIDGETS)
		if (NOT TARGET imp_wxwidgets)
			message(ERROR "imp_wxwidgets should be defined!")
		endif()
		target_link_libraries(${PROJECT_NAME} PRIVATE imp_wxwidgets)
	endif()

	if(CMAKE_MRPT_HAS_Qt5)
		target_link_libraries(${PROJECT_NAME} PRIVATE Qt5::Widgets Qt5::OpenGL)
	endif()

	if(MRPT_OPENGL_LIBS)
		# Link against glut:
		target_link_libraries(${PROJECT_NAME} PRIVATE ${MRPT_OPENGL_LIBS})
	endif()

	if (TARGET mrpt_glew) # Basically, for Windows
		target_link_libraries(${PROJECT_NAME} PRIVATE mrpt_glew)
	endif()

	if (CMAKE_MRPT_HAS_NANOGUI)
		target_link_libraries(${PROJECT_NAME} PUBLIC mrpt::nanogui)
	endif()

	# Use the mrpt::opengl profiler option here too for the opengl GUIs:
	if (MRPT_OPENGL_PROFILER)
		target_compile_definitions(${PROJECT_NAME} PRIVATE MRPT_OPENGL_PROFILER)
	endif()
endif()
