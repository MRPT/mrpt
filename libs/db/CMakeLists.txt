#  See "DeclareMRPTLib.cmake"
list(APPEND db_EXTRA_SRCS		"${MRPT_LIBS_ROOT}/db/src/xmlparser/*.cpp" "${MRPT_LIBS_ROOT}/db/src/xmlparser/*.h")
list(APPEND db_EXTRA_SRCS_NAME 	"db/xmlparser" "db/xmlparser")

# extra dependencies required by unit tests in this module:
#set_property(GLOBAL PROPERTY mrpt_db_UNIT_TEST_EXTRA_DEPS mrpt-XX)

define_mrpt_lib(
	db 		# Lib name
	# Dependencies:
	mrpt-core
	mrpt-serialization
	mrpt-system
	)

if(BUILD_mrpt-db)
	# Ignore precompiled headers in some sources:
	if(MRPT_ENABLE_PRECOMPILED_HDRS)
		if (MSVC)
			set_source_files_properties(
				${db/xmlparser_FILES}
				PROPERTIES COMPILE_FLAGS "/Y-"
				)
		endif ()
	endif()

endif()
