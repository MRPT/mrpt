# National Instruments "DAQmxBase" C API 
# -----------------------------------------

set(MRPT_HAS_NI_DAQmxBASE OFF CACHE BOOL "Build with support for National Instruments DAQmx Base C API")
set(CMAKE_MRPT_HAS_NIDAQMXBASE 0)

# Leave at the user's choice to disable the SWR libs:
option(DISABLE_NationalInstruments "Forces NOT using NationalInstrument libs, even if they are found by CMake" "OFF")
mark_as_advanced(DISABLE_NationalInstruments)
if(NOT DISABLE_NationalInstruments)	
	if(MRPT_HAS_NI_DAQmxBASE)
		# Look for DAQmxBase libs & headers:
		find_library(NI_DAQmxBASE_LIB_FILE
			NAMES nidaqmxbase libnidaqmxbase
			PATHS 
				"/usr/local/natinst/nidaqmxbase"
				"$ENV{ProgramFiles}/National Instruments/NI-DAQmx Base/Lib"
				"$ENV{ProgramFiles(x86)}/National Instruments/NI-DAQmx Base/Lib"
				"$ENV{ProgramW6432}/National Instruments/NI-DAQmx Base/Lib"
			DOC "Full path of library file 'libnidaqmxbase' or 'nidaqmxbase.lib'"
		)
		
		find_path(NI_DAQmxBASE_INCLUDE_DIR
			NAMES NIDAQmxBase.h nidaqmxbase.h
			PATHS 
				"/usr/local/natinst/nidaqmxbase"
				"$ENV{ProgramFiles}/National Instruments/NI-DAQmx Base/Include"
				"$ENV{ProgramFiles(x86)}/National Instruments/NI-DAQmx Base/Include"
				"$ENV{ProgramW6432}/National Instruments/NI-DAQmx Base/Include"
			DOC "Path to [PATH]/NIDAQmxBase.h"
		)
		
		# The variable that actually ends up in config.h
		if (NI_DAQmxBASE_INCLUDE_DIR AND NI_DAQmxBASE_LIB_FILE)
			set(CMAKE_MRPT_HAS_NIDAQMXBASE 1) 
		else(NI_DAQmxBASE_INCLUDE_DIR AND NI_DAQmxBASE_LIB_FILE)
			message("Error: Correct NI_DAQmxBASE_LIB_FILE and NI_DAQmxBASE_INCLUDE_DIR, or uncheck MRPT_HAS_NI_DAQmxBASE")
		endif (NI_DAQmxBASE_INCLUDE_DIR AND NI_DAQmxBASE_LIB_FILE)
	endif(MRPT_HAS_NI_DAQmxBASE)
endif(NOT DISABLE_NationalInstruments)	


# National Instruments "DAQmx" C API 
# -----------------------------------------
set(MRPT_HAS_NI_DAQmx OFF CACHE BOOL "Build with support for National Instruments DAQmx C API")
set(CMAKE_MRPT_HAS_NIDAQMX 0)
if(NOT DISABLE_NationalInstruments)	
	if(MRPT_HAS_NI_DAQmx)
		# Look for DAQmx  libs & headers:
		find_library(NI_DAQmx_LIB_FILE
			NAMES nidaqmx libnidaqmx
			PATHS 
				"/usr/local/natinst/nidaqmx"
				"$ENV{ProgramFiles}/National Instruments/NI-DAQmx/Lib"
				"$ENV{ProgramFiles(x86)}/National Instruments/NI-DAQmx/Lib"
				"$ENV{ProgramW6432}/National Instruments/NI-DAQmx/Lib"
			DOC "Full path of library file 'libnidaqmx' or 'nidaqmx.lib'"
		)
		
		find_path(NI_DAQmx_INCLUDE_DIR
			NAMES NIDAQmx.h nidaqmx.h
			PATHS 
				"/usr/local/natinst/nidaqmx"
				"$ENV{ProgramFiles}/National Instruments/NI-DAQmx/Include"
				"$ENV{ProgramFiles(x86)}/National Instruments/NI-DAQmx/Include"
				"$ENV{ProgramW6432}/National Instruments/NI-DAQmx/Include"
			DOC "Path to [PATH]/NIDAQmx.h"
		)
		
		# The variable that actually ends up in config.h
		if (NI_DAQmx_INCLUDE_DIR AND NI_DAQmx_LIB_FILE)
			set(CMAKE_MRPT_HAS_NIDAQMX 1) 
		else(NI_DAQmx_INCLUDE_DIR AND NI_DAQmx_LIB_FILE)
			message("Error: Correct NI_DAQmx_LIB_FILE and NI_DAQmx_INCLUDE_DIR, or uncheck MRPT_HAS_NI_DAQmx")
		endif (NI_DAQmx_INCLUDE_DIR AND NI_DAQmx_LIB_FILE)
	endif(MRPT_HAS_NI_DAQmx)
endif(NOT DISABLE_NationalInstruments)	
