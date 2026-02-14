#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	containers
	# Dependencies
	mrpt-core
	mrpt-typemeta
	)

# extra dependencies required by unit tests in this module:
set_property(GLOBAL PROPERTY mrpt_containers_UNIT_TEST_EXTRA_DEPS "mrpt-random;mrpt-poses;mrpt-io")

if(BUILD_mrpt-containers)

	if (TARGET mrpt_libfyaml)
            target_link_libraries(containers PRIVATE mrpt_libfyaml)
        endif()

        if (DEFINED LIBFYAML_VERSION)
            string(LENGTH "${LIBFYAML_VERSION}" _len)
            if(_len LESS 5)
              set(LIBFYAML_VERSION "0.${LIBFYAML_VERSION}")
            endif()
            unset(_len)

            VERSION_TO_HEXADECIMAL(MRPT_LIBFYAML_VERSION_HEX ${LIBFYAML_VERSION})
            target_compile_definitions(containers PRIVATE -DMRPT_LIBFYAML_VERSION=${MRPT_LIBFYAML_VERSION_HEX})
            message(STATUS "MRPT_LIBFYAML_VERSION_HEX  : ${MRPT_LIBFYAML_VERSION_HEX}")
        endif()
endif()
