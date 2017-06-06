# ----------------------------------------------------------------------------
# Include the "CPack" package generator
# ----------------------------------------------------------------------------
SET(CMAKE_INSTALL_DEBUG_LIBRARIES 1)
INCLUDE(InstallRequiredSystemLibraries)

SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "The Mobile Robot Programming Toolkit (MRPT)")
SET(CPACK_PACKAGE_VENDOR "Jose Luis Blanco Claraco")
SET(CPACK_PACKAGE_CONTACT "Jose Luis Blanco Claraco <joseluisblancoc@gmail.com>")

SET(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
SET(CPACK_RESOURCE_FILE_WELCOME "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "MRPT is a set of C++ libraries and applications for mobile robot software development.")
FILE(READ ${CPACK_PACKAGE_DESCRIPTION_FILE} CPACK_DESCRIPTION_TEXT)
STRING(REGEX REPLACE "\"" "" CPACK_DESCRIPTION_TEXT ${CPACK_DESCRIPTION_TEXT})  # It seems \" characters break NSIS.

SET(CPACK_PACKAGE_VERSION_MAJOR "${CMAKE_MRPT_VERSION_NUMBER_MAJOR}")
SET(CPACK_PACKAGE_VERSION_MINOR "${CMAKE_MRPT_VERSION_NUMBER_MINOR}")
SET(CPACK_PACKAGE_VERSION_PATCH "${CMAKE_MRPT_VERSION_NUMBER_PATCH}")

SET(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/COPYING")
SET(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README.md")

SET(CPACK_SOURCE_GENERATOR "TGZ")

SET(CPACK_PACKAGE_INSTALL_DIRECTORY "mrpt-${CMAKE_MRPT_VERSION_NUMBER_MAJOR}.${CMAKE_MRPT_VERSION_NUMBER_MINOR}.${CMAKE_MRPT_VERSION_NUMBER_PATCH}" CACHE STRING "Name of the install directory")
MARK_AS_ADVANCED(CPACK_PACKAGE_INSTALL_DIRECTORY)

SET(PACKAGE_INCLUDES_SOURCES ON CACHE BOOL "Include all sources while building packages")
MARK_AS_ADVANCED(PACKAGE_INCLUDES_SOURCES)

IF(WIN32)
	# --------------------------------
	# Packages for Windows
	# --------------------------------
	SET(CPACK_SOURCE_IGNORE_FILES ".svn/;.*~;build;CMakeCache.txt;_CPack_Pakages/;CMakeFiles/;install/;Makefile;*.cmake")
	
	# There is a bug in NSI that does not handle full unix paths properly. Make
	# sure there is at least one set of four (4) backlasshes.
	SET(CPACK_NSIS_MUI_ICON "${CMAKE_CURRENT_SOURCE_DIR}/share/pixmaps\\\\mrpt_icon.ico")
	SET(CPACK_NSIS_MUI_UNIICON "${CMAKE_CURRENT_SOURCE_DIR}/share/pixmaps\\\\mrpt_icon.ico")
	SET(CPACK_PACKAGE_ICON "${CMAKE_CURRENT_SOURCE_DIR}/apps/wx-common\\\\mrpt_logo.png")

	SET(CPACK_NSIS_INSTALLED_ICON_NAME "${CMAKE_CURRENT_SOURCE_DIR}/share/pixmaps\\\\mrpt_icon.ico")

	SET(CPACK_NSIS_HELP_LINK "http:\\\\\\\\www.mrpt.org")
	SET(CPACK_NSIS_URL_INFO_ABOUT "http:\\\\\\\\www.mrpt.org")
	SET(CPACK_NSIS_CONTACT "joseluisblancoc@gmail.com")

	# Add mrpt/bin dir to system PATH
	SET(CPACK_NSIS_MODIFY_PATH ON)

	# Install header and source files:
	# ---------------------------------------------
	IF (PACKAGE_INCLUDES_SOURCES)
		INSTALL(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libs/"
			COMPONENT Library_sources
			DESTINATION libs
			PATTERN ".svn" EXCLUDE
			PATTERN "*~" EXCLUDE)

		INSTALL(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/apps/"
			COMPONENT App_sources
			DESTINATION apps
			PATTERN ".svn" EXCLUDE
			PATTERN "*~" EXCLUDE)
	ENDIF (PACKAGE_INCLUDES_SOURCES)

	INSTALL(FILES
		AUTHORS
		CMakeLists.txt
		README.md
		version_prefix.txt
	DESTINATION .)

	get_property(_str GLOBAL PROPERTY "MRPT_CPACK_PACKAGE_EXECUTABLES")
	SET(CPACK_PACKAGE_EXECUTABLES ${_str}) # --> Set in each apps/*/CMakeLists.txt file

	SET(CPACK_NSIS_MENU_LINKS
	    "doc;Documentation directory;bin;Directory of executables (bin);doc/chm/libMRPT-@CMAKE_MRPT_VERSION_NUMBER_MAJOR@.@CMAKE_MRPT_VERSION_NUMBER_MINOR@.@CMAKE_MRPT_VERSION_NUMBER_PATCH@.chm;MRPT libraries reference (CHM);http://www.mrpt.org/;Online help;doc/mrpt-book.pdf;The MRPT book (PDF)")

	# Force usage of our custom NSIS template:
	SET(CPACK_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/parse-files/")
	
	# File types association:
	SET(CPACK_NSIS_EXTRA_INSTALL_COMMANDS "
		\\\${registerExtension} \\\"$INSTDIR\\\\bin\\\\RawLogViewer.exe\\\" \\\".rawlog\\\" \\\"Robotic Dataset File\\\"
		\\\${registerExtension} \\\"$INSTDIR\\\\bin\\\\SceneViewer3D.exe\\\" \\\".3Dscene\\\" \\\"Robotic 3D scene\\\"
		")
	SET(CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS "
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
	SET(INSTALL_MSVC_REDISTRIBUTABLE "" CACHE FILEPATH "Select an optional vcredist*.exe file to include in the installation")
	MARK_AS_ADVANCED(INSTALL_MSVC_REDISTRIBUTABLE)
	
	if (NOT "${INSTALL_MSVC_REDISTRIBUTABLE}" STREQUAL "")
		if (EXISTS "${INSTALL_MSVC_REDISTRIBUTABLE}")
			GET_FILENAME_COMPONENT(INSTALL_MSVC_REDISTRIBUTABLE_FILENAME "${INSTALL_MSVC_REDISTRIBUTABLE}" NAME_WE)
		
			install(PROGRAMS ${INSTALL_MSVC_REDISTRIBUTABLE} DESTINATION tmp)
			set(CPACK_NSIS_EXTRA_INSTALL_COMMANDS "${CPACK_NSIS_EXTRA_INSTALL_COMMANDS} 
				   ExecWait \\\"$INSTDIR\\\\tmp\\\\${INSTALL_MSVC_REDISTRIBUTABLE_FILENAME}\\\"
				   ")
		endif (EXISTS "${INSTALL_MSVC_REDISTRIBUTABLE}")
	endif (NOT "${INSTALL_MSVC_REDISTRIBUTABLE}" STREQUAL "")

ENDIF(WIN32)
IF(UNIX)
	# ------------------------------------------------------------------
	# Packages for linux: Not supported, use scritps/prepare_* instead
	# ------------------------------------------------------------------
ENDIF(UNIX)

IF(APPLE)
	SET(CPACK_GENERATOR "TGZ;TBZ2;OSXX11")
Endif(APPLE)

INCLUDE(CPack)
