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
	target_include_directories(core PUBLIC
		$<BUILD_INTERFACE:${MRPT_CONFIG_FILE_INCLUDE_DIR}>
		$<INSTALL_INTERFACE:include>
	)

	target_link_libraries(core PRIVATE Threads::Threads) # for WorkerThreads

	# Enforce C++17 in all dependent projects:
	mrpt_lib_target_requires_cpp17(core)

	# Fix a MSVC binary-breaking compatibility in MSVC 2017 15.8:
	if (MSVC)
		target_compile_definitions(core PUBLIC -D_ENABLE_EXTENDED_ALIGNED_STORAGE)
	endif()

	if(WIN32)
		target_link_libraries(core PRIVATE Dbghelp) # For mrpt::demangle(), mrpt::callStackBackTrace()
	else()
		target_link_libraries(core PRIVATE ${CMAKE_DL_LIBS}) # For mrpt::callStackBackTrace()
	endif()

	if (CMAKE_MRPT_HAS_BFD)
		target_link_libraries(core PRIVATE ${BFD_LIBRARY})
	endif()

endif()
