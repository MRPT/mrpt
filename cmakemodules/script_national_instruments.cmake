# National Instruments "DAQmxBase" C API 
# -----------------------------------------

SET( MRPT_HAS_NI_DAQmxBASE OFF CACHE BOOL "Build with support for National Instruments DAQmx Base C API")

# Leave at the user's choice to disable the SWR libs:
OPTION(DISABLE_NationalInstruments "Forces NOT using NationalInstrument libs, even if they are found by CMake" "OFF")
MARK_AS_ADVANCED(DISABLE_NationalInstruments)
IF(NOT DISABLE_NationalInstruments)	
	IF(MRPT_HAS_NI_DAQmxBASE)
		# Look for DAQmxBase libs & headers:
		SET(NI_DAQmxBASE_ROOT_DIR "" CACHE PATH "Path to NI root directory. Typ: WIN=C:\\Program Files\\National Instruments\\NI-DAQmx Base . LIN=/usr/local/")
		SET(CMAKE_MRPT_HAS_NIDAQMXBASE 0) # The variable that actually ends up in config.h
		
		# Checks:
		SET(NI_DAQmxBASE_LIB CACHE FILE "" "Library file nidaqmxbase.{lib|so}")
		IF (EXISTS "${NI_DAQmxBASE_ROOT_DIR}/lib/nidaqmxbase.lib")
			SET(NI_DAQmxBASE_LIB "${NI_DAQmxBASE_ROOT_DIR}/lib/nidaqmxbase.lib")
		ENDIF (EXISTS "${NI_DAQmxBASE_ROOT_DIR}/lib/nidaqmxbase.lib")

		SET(NI_DAQmxBASE_INCLUDE_DIR CACHE PATH "" "Path to NIDAQmxBase.h")
		IF (EXISTS "${NI_DAQmxBASE_ROOT_DIR}/include/NIDAQmxBase.h")
			SET(NI_DAQmxBASE_INCLUDE_DIR "${NI_DAQmxBASE_ROOT_DIR}/include/")
		ENDIF (EXISTS "${NI_DAQmxBASE_ROOT_DIR}/include/NIDAQmxBase.h")
						
		# The variable that actually ends up in config.h
		IF (EXISTS "${NI_DAQmxBASE_LIB}" AND EXISTS "${NI_DAQmxBASE_INCLUDE_DIR}/NIDAQmxBase.h")
			SET(CMAKE_MRPT_HAS_NIDAQMXBASE 1) 
		ENDIF (EXISTS "${NI_DAQmxBASE_LIB}" AND EXISTS "${NI_DAQmxBASE_INCLUDE_DIR}/NIDAQmxBase.h")
			
		IF(NOT CMAKE_MRPT_HAS_NIDAQMXBASE)
			MESSAGE("Error: Set NI_DAQmxBASE_ROOT_DIR correctly (to contain 'NI_DAQmxBASE_ROOT_DIR/include/NIDAQmxBase.h'), or uncheck MRPT_HAS_NI_DAQmxBASE")
		ENDIF(NOT CMAKE_MRPT_HAS_NIDAQMXBASE)
		
	ENDIF(MRPT_HAS_NI_DAQmxBASE)
ENDIF(NOT DISABLE_NationalInstruments)	
