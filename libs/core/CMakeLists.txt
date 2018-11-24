#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	core
	# Dependencies
	)

if(BUILD_mrpt-core)
	# Includes: <mrpt/config.h> & <mrpt/version.h> config headers:
	target_include_directories(mrpt-core PUBLIC
		$<BUILD_INTERFACE:${MRPT_CONFIG_FILE_INCLUDE_DIR}>
		$<INSTALL_INTERFACE:${libmrpt_common_dev_INSTALL_PREFIX}include>
	)

	# Enforce C++17 in all dependent projects:
	mrpt_lib_target_requires_cpp17(mrpt-core)

	# Fix a MSVC binary-breaking compatibility in MSVC 2017 15.8:
	if (MSVC)
		target_compile_definitions(mrpt-core PUBLIC -D_ENABLE_EXTENDED_ALIGNED_STORAGE)
	endif()
endif()
