# Support for SWISSRANGE 3D camera:
# ===================================================
set(CMAKE_MRPT_HAS_SWISSRANGE 0)
set(CMAKE_MRPT_HAS_SWISSRANGE_SYSTEM 0)

if(UNIX)
	# Linux: Look for the libMesaSR.h include:
	find_file(SWR_HEADER_FILE  libMesaSR.h)
	if(SWR_HEADER_FILE)
		set(CMAKE_MRPT_HAS_SWISSRANGE 1)
		mark_as_advanced(SWR_HEADER_FILE)
		set(MRPT_SWR_LIBS mesasr) #APPEND_MRPT_LIBS(mesasr)
		
	else(SWR_HEADER_FILE)
		set(CMAKE_MRPT_HAS_SWISSRANGE 0)
	endif(SWR_HEADER_FILE)
else(UNIX)
	if (MSVC)
		# Windows: ...
		find_path(SWR_LIBMESASR_DIR  MesaImaging/Swissranger/libMesaSR/)

		if(SWR_LIBMESASR_DIR)
			set(SWR_LIBMESASR_DIR "${SWR_LIBMESASR_DIR}/MesaImaging/Swissranger/libMesaSR/")
			message(STATUS "SwissRanger Library found in: ${SWR_LIBMESASR_DIR}")

			# We expect to find there "libMesaSR.lib" & "libMesaSR.h"
			if (EXISTS "${SWR_LIBMESASR_DIR}/libMesaSR.lib" AND EXISTS "${SWR_LIBMESASR_DIR}/libMesaSR.h")
				set(CMAKE_MRPT_HAS_SWISSRANGE 1)
				include_directories("${SWR_LIBMESASR_DIR}")
				#link_directories("${SWR_LIBMESASR_DIR}")
				set(MRPT_SWR_LIBS "${SWR_LIBMESASR_DIR}/libMesaSR.lib") #APPEND_MRPT_LIBS("${SWR_LIBMESASR_DIR}/libMesaSR.lib")
				mark_as_advanced(SWR_LIBMESASR_DIR)
			else (EXISTS "${SWR_LIBMESASR_DIR}/libMesaSR.lib" AND EXISTS "${SWR_LIBMESASR_DIR}/libMesaSR.h")
				message(STATUS "*** ERROR *** SwissRanger Library directory found but doesn't contain expected files. Not using it.")
				set(CMAKE_MRPT_HAS_SWISSRANGE 0)
			endif (EXISTS "${SWR_LIBMESASR_DIR}/libMesaSR.lib" AND EXISTS "${SWR_LIBMESASR_DIR}/libMesaSR.h")
		else(SWR_LIBMESASR_DIR)
			set(CMAKE_MRPT_HAS_SWISSRANGE 0)
		endif(SWR_LIBMESASR_DIR)
	endif(MSVC)
endif(UNIX)

# Leave at the user's choice to disable the SWR libs:
option(DISABLE_SWISSRANGER_3DCAM_LIBS "Disable the usage (if found) of SWR libs" "OFF")
mark_as_advanced(DISABLE_SWISSRANGER_3DCAM_LIBS)
if(DISABLE_SWISSRANGER_3DCAM_LIBS)
	set(CMAKE_MRPT_HAS_SWISSRANGE 0)
endif(DISABLE_SWISSRANGER_3DCAM_LIBS)

# Can only be a system lib:
set(CMAKE_MRPT_HAS_SWISSRANGE_SYSTEM 0)
if(CMAKE_MRPT_HAS_SWISSRANGE)
	set(CMAKE_MRPT_HAS_SWISSRANGE_SYSTEM 1)
endif(CMAKE_MRPT_HAS_SWISSRANGE)

