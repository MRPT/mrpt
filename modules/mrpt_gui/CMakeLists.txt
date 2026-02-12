
cmake_minimum_required(VERSION 3.16)
project(mrpt_gui LANGUAGES C CXX)

# Ensure core MRPT packages used here are available to CMake
find_package(mrpt_common REQUIRED)
find_package(mrpt_opengl REQUIRED)

include(./cmake/script_nanogui.cmake REQUIRED)
include(./cmake/script_qt.cmake REQUIRED)
include(./cmake/script_wxwidgets.cmake REQUIRED)

if (CMAKE_MRPT_HAS_NANOGUI)
	set(nanogui_dep mrpt-nanogui)
else()
	set(nanogui_dep "")
endif()

# define lib:
set(LIB_SOURCES
  src/about_box_qt.cpp
  src/about_box_wx.cpp
  src/CAboutBox_wx.cpp
  src/CAboutBox_wx.h
  src/CAboutBoxBase.cpp
  src/CAboutBoxBase.h
  src/CAboutBoxQt.cpp
  src/CAboutBoxQt.h
  src/CBaseGUIWindow.cpp
  src/CDisplayWindow.cpp
  src/CDisplayWindow3D.cpp
  src/CDisplayWindowGUI.cpp
  src/CDisplayWindowPlots.cpp
  src/CGlCanvasBase.cpp
  src/CQtGlCanvasBase.cpp
  src/CWxGLCanvasBase.cpp
  src/default_mrpt_glfw_icon.cpp
  src/error_box.cpp
  src/mathplots/mathplot.cpp
  src/mathplots/README
  src/mathplots/README.txt
  src/MRPT2NanoguiGLCanvas.cpp
  src/NanoGUICanvasHeadless.cpp
  src/registerAllClasses.cpp
  src/WxSubsystem.cpp
  src/WxUtils.cpp
)
set(LIB_PUBLIC_HEADERS
  include/mrpt/3rdparty/mathplot/mathplot.h
  include/mrpt/gui.h
  include/mrpt/gui/about_box.h
  include/mrpt/gui/CBaseGUIWindow.h
  include/mrpt/gui/CDisplayWindow.h
  include/mrpt/gui/CDisplayWindow3D.h
  include/mrpt/gui/CDisplayWindowGUI.h
  include/mrpt/gui/CDisplayWindowPlots.h
  include/mrpt/gui/CGlCanvasBase.h
  include/mrpt/gui/CMyRedirector.h
  include/mrpt/gui/CQtGlCanvasBase.h
  include/mrpt/gui/CWxGLCanvasBase.h
  include/mrpt/gui/default_mrpt_glfw_icon.h
  include/mrpt/gui/error_box.h
  include/mrpt/gui/gui_frwds.h
  include/mrpt/gui/internal/NanoGUICanvasHeadless.h
  include/mrpt/gui/keycodes.h
  include/mrpt/gui/MRPT2NanoguiGLCanvas.h
  include/mrpt/gui/registerAllClasses.h
  include/mrpt/gui/wx28-fixes.h
  include/mrpt/gui/WxSubsystem.h
  include/mrpt/gui/WxUtils.h
)

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_opengl
    ${nanogui_dep}
  CMAKE_DEPENDENCIES
    mrpt_opengl
)

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
