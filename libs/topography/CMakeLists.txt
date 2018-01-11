#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	topography
	# Dependencies
	mrpt-obs
	)

IF(BUILD_mrpt-topography)
	IF(CMAKE_MRPT_HAS_WXWIDGETS)
		target_link_libraries(mrpt-topography PRIVATE ${wxWidgets_LIBRARIES})
	ENDIF()
ENDIF()
