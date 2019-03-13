
list(APPEND poses_EXTRA_SRCS		"${MRPT_SOURCE_DIR}/libs/poses/src/Lie/*.cpp" "${MRPT_SOURCE_DIR}/libs/poses/include/mrpt/poses/Lie/*.h")
list(APPEND poses_EXTRA_SRCS_NAME 	"Lie" "Lie")


define_mrpt_lib(
	poses 		# Lib name
	# Dependencies:
	mrpt-bayes
	)

if(BUILD_mrpt-poses)

endif()
