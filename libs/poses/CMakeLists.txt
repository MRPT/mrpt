
list(APPEND poses_EXTRA_SRCS		"${MRPT_SOURCE_DIR}/libs/poses/src/Lie/*.cpp" "${MRPT_SOURCE_DIR}/libs/poses/include/mrpt/poses/Lie/*.h")
list(APPEND poses_EXTRA_SRCS_NAME 	"Lie" "Lie")


define_mrpt_lib(
	poses 		# Lib name
	# Dependencies:
	mrpt-bayes
	)

if(BUILD_mrpt-poses)

	# Windows only: the mxplus lib internally requires linking with opencv (sigh...)
	# Perhaps there is a better way to solve this, someday.
	if (MRPT_WITH_MATLAB_WRAPPER)
		target_link_libraries(poses PRIVATE imp_opencv)
	endif()

endif()
