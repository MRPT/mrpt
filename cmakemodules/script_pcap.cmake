# Check for the libpcap library 
# ==============================

option(DISABLE_PCAP "Disable using the PCAP library" "OFF")
mark_as_advanced(DISABLE_PCAP)

set(CMAKE_MRPT_HAS_LIBPCAP 0)
set(CMAKE_MRPT_HAS_LIBPCAP_SYSTEM 0)

if (NOT DISABLE_PCAP)
	find_package(PCAP QUIET)
	if(PCAP_FOUND)
		set(CMAKE_MRPT_HAS_LIBPCAP 1)
		set(CMAKE_MRPT_HAS_LIBPCAP_SYSTEM 1)

		if ($ENV{VERBOSE})
			message(STATUS "Found libpcap:")
			message(STATUS "  PCAP_INCLUDE_DIR :${PCAP_INCLUDE_DIR}")
			message(STATUS "  PCAP_LIBRARY     :${PCAP_LIBRARY}")
		endif()

	endif()
endif()

