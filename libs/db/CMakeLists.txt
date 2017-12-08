#  See "DeclareMRPTLib.cmake"
LIST(APPEND db_EXTRA_SRCS		"${MRPT_LIBS_ROOT}/db/src/xmlparser/*.cpp" "${MRPT_LIBS_ROOT}/db/src/xmlparser/*.h")
LIST(APPEND db_EXTRA_SRCS_NAME 	"db/xmlparser" "db/xmlparser")

define_mrpt_lib(
	db 		# Lib name
	# Dependencies:
	mrpt-core
	mrpt-serialization
	)

IF(BUILD_mrpt-db)
	# Ignore precompiled headers in some sources:
	IF(MRPT_ENABLE_PRECOMPILED_HDRS)
		IF (MSVC)
			set_source_files_properties(
				${db/xmlparser_FILES}
				PROPERTIES COMPILE_FLAGS "/Y-"
				)
		ENDIF ()
	ENDIF()

ENDIF()
