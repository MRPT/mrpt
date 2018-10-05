# ----------------------------------------------------------------------------
# Include the "CPack" package generator
# ----------------------------------------------------------------------------
set(CMAKE_INSTALL_DEBUG_LIBRARIES 1)
include(InstallRequiredSystemLibraries)

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "The Mobile Robot Programming Toolkit (MRPT)")
set(CPACK_PACKAGE_VENDOR "Jose Luis Blanco Claraco")
set(CPACK_PACKAGE_CONTACT "Jose Luis Blanco Claraco <joseluisblancoc@gmail.com>")

set(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
set(CPACK_RESOURCE_FILE_WELCOME "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "MRPT is a set of C++ libraries and applications for mobile robot software development.")
file(READ ${CPACK_PACKAGE_DESCRIPTION_FILE} CPACK_DESCRIPTION_TEXT)
string(REGEX REPLACE "\"" "" CPACK_DESCRIPTION_TEXT ${CPACK_DESCRIPTION_TEXT})  # It seems \" characters break NSIS.

set(CPACK_PACKAGE_VERSION_MAJOR "${CMAKE_MRPT_VERSION_NUMBER_MAJOR}")
set(CPACK_PACKAGE_VERSION_MINOR "${CMAKE_MRPT_VERSION_NUMBER_MINOR}")
set(CPACK_PACKAGE_VERSION_PATCH "${CMAKE_MRPT_VERSION_NUMBER_PATCH}")

set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/COPYING")
set(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README.md")

set(CPACK_SOURCE_GENERATOR "TGZ")

set(CPACK_PACKAGE_INSTALL_DIRECTORY "mrpt-${CMAKE_MRPT_VERSION_NUMBER_MAJOR}.${CMAKE_MRPT_VERSION_NUMBER_MINOR}.${CMAKE_MRPT_VERSION_NUMBER_PATCH}" CACHE STRING "Name of the install directory")
mark_as_advanced(CPACK_PACKAGE_INSTALL_DIRECTORY)

if(WIN32)
	# --------------------------------
	# Packages for Windows
	# --------------------------------
	set(CPACK_SOURCE_IGNORE_FILES ".svn/;.*~;build;CMakeCache.txt;_CPack_Pakages/;CMakeFiles/;install/;Makefile;*.cmake")

	# There is a bug in NSI that does not handle full unix paths properly. Make
	# sure there is at least one set of four (4) backlasshes.
	set(CPACK_NSIS_MUI_ICON "${CMAKE_CURRENT_SOURCE_DIR}/share/pixmaps\\\\mrpt_icon.ico")
	set(CPACK_NSIS_MUI_UNIICON "${CMAKE_CURRENT_SOURCE_DIR}/share/pixmaps\\\\mrpt_icon.ico")
	set(CPACK_PACKAGE_ICON "${CMAKE_CURRENT_SOURCE_DIR}/apps/wx-common\\\\mrpt_logo.png")

	set(CPACK_NSIS_INSTALLED_ICON_NAME "${CMAKE_CURRENT_SOURCE_DIR}/share/pixmaps\\\\mrpt_icon.ico")

	set(CPACK_NSIS_HELP_LINK "http:\\\\\\\\www.mrpt.org")
	set(CPACK_NSIS_URL_INFO_ABOUT "http:\\\\\\\\www.mrpt.org")
	set(CPACK_NSIS_CONTACT "joseluisblancoc@gmail.com")

	# Add mrpt/bin dir to system PATH
	set(CPACK_NSIS_MODIFY_PATH ON)

	# Install header and source files:
	# ---------------------------------------------
	install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libs/"
		COMPONENT Library_sources
		DESTINATION libs
		PATTERN "*~" EXCLUDE)

	install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/apps/"
		COMPONENT App_sources
		DESTINATION apps
		PATTERN "*~" EXCLUDE)

	install(FILES
		AUTHORS
		CMakeLists.txt
		README.md
		version_prefix.txt
	DESTINATION .)

	get_property(_str GLOBAL PROPERTY "MRPT_CPACK_PACKAGE_EXECUTABLES")
	set(CPACK_PACKAGE_EXECUTABLES ${_str}) # --> Set in each apps/*/CMakeLists.txt file

	set(CPACK_NSIS_MENU_LINKS
	    "doc;Documentation directory;bin;Directory of executables (bin);doc/chm/libMRPT-@CMAKE_MRPT_VERSION_NUMBER_MAJOR@.@CMAKE_MRPT_VERSION_NUMBER_MINOR@.@CMAKE_MRPT_VERSION_NUMBER_PATCH@.chm;MRPT libraries reference (CHM);http://www.mrpt.org/;Online help;doc/mrpt-book.pdf;The MRPT book (PDF)")

	# Force usage of our custom NSIS template:
	set(CPACK_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/parse-files/")

	# File types association:
	set(CPACK_NSIS_EXTRA_INSTALL_COMMANDS "
		\\\${registerExtension} \\\"$INSTDIR\\\\bin\\\\RawLogViewer.exe\\\" \\\".rawlog\\\" \\\"Robotic Dataset File\\\"
		\\\${registerExtension} \\\"$INSTDIR\\\\bin\\\\SceneViewer3D.exe\\\" \\\".3Dscene\\\" \\\"Robotic 3D scene\\\"
		")
	set(CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS "
		\\\${unregisterExtension} \\\".rawlog\\\" \\\"Robotic Dataset File\\\"
		\\\${unregisterExtension} \\\".3Dscene\\\" \\\"Robotic 3D scene\\\"
		")

	# Install to "Program files (x86)" or "Program files" correctly:
	if(CMAKE_SIZEOF_VOID_P EQUAL 8)
	  set(CPACK_NSIS_PROGRAMFILES PROGRAMFILES64)
	else(CMAKE_SIZEOF_VOID_P EQUAL 8)
	  set(CPACK_NSIS_PROGRAMFILES PROGRAMFILES)
	endif(CMAKE_SIZEOF_VOID_P EQUAL 8)

	# Allow installing VC redistributables:
	set(INSTALL_MSVC_REDISTRIBUTABLE "" CACHE FILEPATH "Select an optional vcredist*.exe file to include in the installation")
	mark_as_advanced(INSTALL_MSVC_REDISTRIBUTABLE)

	if (NOT "${INSTALL_MSVC_REDISTRIBUTABLE}" STREQUAL "")
		if (EXISTS "${INSTALL_MSVC_REDISTRIBUTABLE}")
			get_filename_component(INSTALL_MSVC_REDISTRIBUTABLE_FILENAME "${INSTALL_MSVC_REDISTRIBUTABLE}" NAME_WE)

			install(PROGRAMS ${INSTALL_MSVC_REDISTRIBUTABLE} DESTINATION tmp)
			set(CPACK_NSIS_EXTRA_INSTALL_COMMANDS "${CPACK_NSIS_EXTRA_INSTALL_COMMANDS}
				   ExecWait \\\"$INSTDIR\\\\tmp\\\\${INSTALL_MSVC_REDISTRIBUTABLE_FILENAME}\\\"
				   ")
		endif (EXISTS "${INSTALL_MSVC_REDISTRIBUTABLE}")
	endif (NOT "${INSTALL_MSVC_REDISTRIBUTABLE}" STREQUAL "")

endif(WIN32)
if(UNIX)
	# ------------------------------------------------------------------
	# Packages for linux: Not supported, use scritps/prepare_* instead
	# ------------------------------------------------------------------
endif(UNIX)

if(APPLE)
	set(CPACK_GENERATOR "TGZ;TBZ2;OSXX11")
Endif(APPLE)

include(CPack)
