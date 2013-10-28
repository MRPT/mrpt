# National Instruments "DAQmxBase" C API 
# -----------------------------------------

SET( MRPT_HAS_NI_DAQmxBASE OFF CACHE BOOL "Build with support for National Instruments DAQmx Base C API")

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
			SET(CMAKE_MRPT_HAS_NIDAQMXBASE 0)
			MESSAGE("Error: Correct NI_DAQmxBASE_LIB_FILE and NI_DAQmxBASE_INCLUDE_DIR, or uncheck MRPT_HAS_NI_DAQmxBASE")
		ENDIF (NI_DAQmxBASE_INCLUDE_DIR AND NI_DAQmxBASE_LIB_FILE)
			
	ENDIF(MRPT_HAS_NI_DAQmxBASE)
ENDIF(NOT DISABLE_NationalInstruments)	
