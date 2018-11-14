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
	if(${CMAKE_VERSION} VERSION_LESS "3.8.0")
		# Support older cmake versions:
		if (MSVC)
			target_compile_options(mrpt-core PUBLIC "/std:c++latest")
		else()
			target_compile_options(mrpt-core PUBLIC "-std=c++17")
		endif()
	else()
		if (NOT MSVC)
			# Modern, clean way to do this:
			target_compile_features(mrpt-core PUBLIC cxx_std_17)
		else()
			# At present (CMake 3.12 + MSVC 19.15.26732.1) it seems cxx_std_17
			# does not enable C++17 in MSVC (!).
			target_compile_options(mrpt-core PUBLIC "/std:c++17")
		endif()
	endif()

	# Fix a MSVC binary-breaking compatibility in MSVC 2017 15.8:
	if (MSVC)
		target_compile_definitions(mrpt-core PUBLIC -D_ENABLE_EXTENDED_ALIGNED_STORAGE)
	endif()
endif()
