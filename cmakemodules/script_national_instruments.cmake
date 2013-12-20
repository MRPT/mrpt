# National Instruments "DAQmxBase" C API 
# -----------------------------------------

SET(MRPT_HAS_NI_DAQmxBASE OFF CACHE BOOL "Build with support for National Instruments DAQmx Base C API")
SET(CMAKE_MRPT_HAS_NIDAQMXBASE 0)

# Leave at the user's choice to disable the SWR libs:
OPTION(DISABLE_NationalInstruments "Forces NOT using NationalInstrument libs, even if they are found by CMake" "OFF")
MARK_AS_ADVANCED(DISABLE_NationalInstruments)
IF(NOT DISABLE_NationalInstruments)	
	IF(MRPT_HAS_NI_DAQmxBASE)
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
		IF (NI_DAQmxBASE_INCLUDE_DIR AND NI_DAQmxBASE_LIB_FILE)
			SET(CMAKE_MRPT_HAS_NIDAQMXBASE 1) 
		ELSE(NI_DAQmxBASE_INCLUDE_DIR AND NI_DAQmxBASE_LIB_FILE)
			MESSAGE("Error: Correct NI_DAQmxBASE_LIB_FILE and NI_DAQmxBASE_INCLUDE_DIR, or uncheck MRPT_HAS_NI_DAQmxBASE")
		ENDIF (NI_DAQmxBASE_INCLUDE_DIR AND NI_DAQmxBASE_LIB_FILE)
	ENDIF(MRPT_HAS_NI_DAQmxBASE)
ENDIF(NOT DISABLE_NationalInstruments)	


# National Instruments "DAQmx" C API 
# -----------------------------------------
SET(MRPT_HAS_NI_DAQmx OFF CACHE BOOL "Build with support for National Instruments DAQmx C API")
SET(CMAKE_MRPT_HAS_NIDAQMX 0)
IF(NOT DISABLE_NationalInstruments)	
	IF(MRPT_HAS_NI_DAQmx)
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
		IF (NI_DAQmx_INCLUDE_DIR AND NI_DAQmx_LIB_FILE)
			SET(CMAKE_MRPT_HAS_NIDAQMX 1) 
		ELSE(NI_DAQmx_INCLUDE_DIR AND NI_DAQmx_LIB_FILE)
			MESSAGE("Error: Correct NI_DAQmx_LIB_FILE and NI_DAQmx_INCLUDE_DIR, or uncheck MRPT_HAS_NI_DAQmx")
		ENDIF (NI_DAQmx_INCLUDE_DIR AND NI_DAQmx_LIB_FILE)
	ENDIF(MRPT_HAS_NI_DAQmx)
ENDIF(NOT DISABLE_NationalInstruments)	
