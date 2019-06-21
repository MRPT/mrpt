#  See "DeclareMRPTLib.cmake"
list(APPEND config_EXTRA_SRCS		"${MRPT_LIBS_ROOT}/config/*.cpp" "${MRPT_LIBS_ROOT}/config/*.h")
list(APPEND config_EXTRA_SRCS_NAME 	"config" "config")

list(APPEND config_EXTRA_SRCS       "${MRPT_LIBS_ROOT}/config/src/simpleini/*.h")
list(APPEND config_EXTRA_SRCS_NAME  "config/simpleini")

define_mrpt_lib(
	config 		# Lib name
	# Dependencies:
	mrpt-core
	mrpt-system
	mrpt-expr
	)

if(BUILD_mrpt-config)
	target_link_libraries(config PRIVATE ${LIBYAMLCPP_LIBRARIES})
	target_include_directories(config PRIVATE ${LIBYAMLCPP_INCLUDE_DIRS})
endif()
