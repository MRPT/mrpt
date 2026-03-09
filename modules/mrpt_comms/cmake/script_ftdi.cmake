# Check for the FTDI headers (Linux/Unix only; on Windows we use a built-in
# header and dynamic DLL loading at runtime).
# ===================================================
set(CMAKE_MRPT_HAS_FTDI 0)

option(DISABLE_FTDI "Do not use the USB driver for FTDI chips" OFF)
mark_as_advanced(DISABLE_FTDI)

if(DISABLE_FTDI)
  return()
endif()

if(UNIX)
  # --- Attempt 1: modern cmake find-module for libftdi1 (>= 1.2) ---
  find_package(LibFTDI1 QUIET)
  if(LibFTDI1_FOUND)
    set(CMAKE_MRPT_HAS_FTDI 1)
    set(CMAKE_MRPT_HAS_FTDI_SYSTEM 1)
    set(FTDI_INCLUDE_DIRS  ${LIBFTDI_INCLUDE_DIRS})
    set(FTDI_LINK_DIRS     ${LIBFTDI_LIBRARY_DIRS})
    set(FTDI_LIBS          ${LIBFTDI_LIBRARIES})
  endif()

  # --- Attempt 2: pkg-config for libftdi1 or the older libftdi ---
  if(NOT CMAKE_MRPT_HAS_FTDI)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
      # Try libftdi1 first, then the older libftdi package name
      pkg_check_modules(FTDI QUIET libftdi1)
      if(NOT FTDI_FOUND)
        pkg_check_modules(FTDI QUIET libftdi)
      endif()
      if(FTDI_FOUND)
        set(CMAKE_MRPT_HAS_FTDI 1)
        set(CMAKE_MRPT_HAS_FTDI_SYSTEM 1)
        # pkg_check_modules populates FTDI_INCLUDE_DIRS, FTDI_LIBRARY_DIRS,
        # FTDI_LIBRARIES (link flags without -l/-L) automatically.
        set(FTDI_LINK_DIRS ${FTDI_LIBRARY_DIRS})
        set(FTDI_LIBS      ${FTDI_LIBRARIES})
      endif()
    endif()
  endif()

  # --- Create an INTERFACE imported target for consumers ---
  if(CMAKE_MRPT_HAS_FTDI)
    if($ENV{VERBOSE})
      message(STATUS "libftdi configuration:")
      message(STATUS "  FTDI_INCLUDE_DIRS : ${FTDI_INCLUDE_DIRS}")
      message(STATUS "  FTDI_LINK_DIRS    : ${FTDI_LINK_DIRS}")
      message(STATUS "  FTDI_LIBS         : ${FTDI_LIBS}")
    endif()

    add_library(imp_ftdi INTERFACE IMPORTED)
    set_target_properties(imp_ftdi PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${FTDI_INCLUDE_DIRS}"
      INTERFACE_LINK_DIRECTORIES    "${FTDI_LINK_DIRS}"
      INTERFACE_LINK_LIBRARIES      "${FTDI_LIBS}"
    )
  endif()

else()
  # On Windows, FTDI support is always compiled in (header bundled, DLL
  # loaded at runtime).
  set(CMAKE_MRPT_HAS_FTDI 1)
endif()
