#  See "DeclareMRPTLib.cmake"
list(APPEND config_EXTRA_SRCS		"${MRPT_LIBS_ROOT}/config/*.cpp" "${MRPT_LIBS_ROOT}/config/*.h")
list(APPEND config_EXTRA_SRCS_NAME 	"config" "config")

define_mrpt_lib(
	config 		# Lib name
	# Dependencies:
	mrpt-core
	mrpt-system
	mrpt-expr
	)

if(BUILD_mrpt-config)
	target_include_directories(config PRIVATE "${SIMPLEINI_INCLUDE_DIRS}")
	if (SIMPLEINI_LIBRARY)
	  target_link_libraries(config PRIVATE ${SIMPLEINI_LIBRARY})
	endif()
endif()
